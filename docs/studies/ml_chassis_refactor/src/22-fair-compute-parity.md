# Fair compute parity

Chapter 21 audited how the five RL algorithm implementations in
`sim/L0/ml-bridge/src/` handle `TrainingBudget::Steps(N)` and
reported three things: the Steps-to-epochs formula is uniform,
all five algorithms run the computed number of epochs as a hard
count, and the `total_steps` reporting diverges between on-policy
(sum of trajectory lengths) and off-policy (inner-loop iterations
times `n_envs`, with an all-complete early break). It also
flagged PPO's `k_passes` as a wall-clock factor distinct from
env-step budget, and the TD3/SAC warmup overhead as a small
unbudgeted cost that is invisible in current reporting. Chapter
21 explicitly did not decide how to reconcile any of these. This
chapter does.

Chapter 22's job is to answer five design questions for the
rematch: what unit is "fair compute," what rematch configuration
preserves the uniformity chapter 21 established, how the
on-policy versus off-policy env-work disparity should be
handled in the writeup, what to do about the TD3/SAC warmup
overhead, and how to frame PPO's wall-clock cost. The answers
have to be defensible under peer-review scrutiny and
implementable without rewriting the algorithm surface, because
the rematch is a use of the existing code, not a refactor of
it. Where a design call would require a code change, this
chapter distinguishes between what the rematch can do
immediately (using the current code as-is) and what the proper
long-term fix looks like, and names the second as a
recommendation rather than a dependency.

## The budget unit: env steps

The first decision is the easiest and the recon already names
it. The rematch uses env steps as the compute-parity unit,
expressed through `TrainingBudget::Steps(16_000_000)`. The two
alternatives worth naming and dispatching are wall-clock time
and gradient updates.

**Wall-clock time** has the virtue of being what a user of the
algorithms actually experiences: "I waited an hour for PPO, I
waited forty minutes for CEM." It has the disqualifying flaw of
being a function of the specific machine the rematch runs on.
A rematch comparison run on a 16-core workstation does not
generalize to a 64-core server or a laptop; the winners and
losers can reorder based on how well each algorithm's inner
loop parallelizes on the specific hardware. For a rematch whose
purpose is to establish which algorithm produces the best
result under a fixed amount of physics work, machine-dependent
metrics are a category mistake. Wall-clock belongs in a separate
performance-engineering chapter, not in the rematch protocol.

**Gradient updates** has the virtue of measuring "how much
learning work did the algorithm do." It has the disqualifying
flaw of being non-uniform across algorithm families — CEM does
zero gradient updates (it is population-based and uses no
backprop); REINFORCE and PPO do one gradient update per epoch
plus PPO's `k_passes` multiplier; TD3 and SAC do one gradient
update per inner-loop step after warmup, plus SAC's additional
temperature update. A rematch that tried to equalize "gradient
updates" across CEM and TD3 would be comparing zero to tens of
thousands, and the equalizing constraint would collapse. The
unit is not defined uniformly enough to be a compute-parity
mechanism.

**Physics-simulation seconds** is the most plausible fourth
alternative — "how much MuJoCo rollout time did each algorithm
consume" — and is worth naming explicitly because in a
thermo-computing study whose entire substrate is physics
simulation, a referee will ask about it. It collapses to env
steps at fixed `dt` and fixed `max_episode_steps`: one env step
corresponds to one integrator step of duration `dt`, so
(env_steps × dt) is sim-seconds, and at uniform hyperparameters
across the rematch surface the conversion factor is a constant.
Sim-seconds is env steps expressed in different units; picking
one or the other is a presentation choice, not a design choice.
Env steps is the cheaper expression of the same quantity and
is what the rematch writeup reports.

**Env steps** is the standard RL answer, is the unit the
existing `TrainingBudget` enum already supports via its `Steps`
variant, and is the unit chapter 21 established as uniform at
the budget-enforcement level. Every algorithm receives the
same `Steps(N)` budget, converts it through the same formula to
the same epoch count, and runs that many epochs as a hard
iteration limit. The budget is the same for all five
algorithms. What each epoch actually *does* inside itself is
where the next section's problem lives, and that problem is
deeper than chapter 21's "reporting divergence" framing made it
sound.

Env steps is therefore the rematch budget unit. `Steps(16M)` is
the specific value, and the rematch runs all five algorithms
with identical `n_envs = 32` and `max_episode_steps = 5000` so
that the `Steps`-to-epochs formula produces the same 100 epochs
for every algorithm. The rationale for the unit is budget-level
consistency across the algorithm surface at the epoch-count
level, agreement with the standard RL literature convention,
and that the realizable alternatives (wall-clock,
gradient updates, sim-seconds) each either fail the uniformity
test or collapse to the same number under different units.

## The work inside an epoch is not uniform

Chapter 21 established that on-policy `total_steps` is a literal
sum of trajectory lengths and off-policy `total_steps` is
inner-loop iterations times `n_envs`, with the all-complete
early break at `td3.rs:273–275` and `sac.rs:300–302` bounding
the iteration count. Chapter 21 framed this as a reporting
divergence. Reading the off-policy inner loop carefully — in
particular `td3.rs:272–310` — makes clear that the framing
understates what is happening. Every iteration of the
off-policy inner loop calls `env.step()` on *all* `n_envs`
envs unconditionally, regardless of whether any of them has
completed an episode during the current epoch. A completed env
is auto-reset to a new starting state and then continues being
stepped through the remainder of the inner loop. The inner
loop exits only when the slowest env has completed at least
one episode — at which point the faster envs have already
been stepped well past their first termination.

The practical consequence is that both counters are honest:
each counts the literal number of `env.step()` calls its family
performs in an epoch. And at the same `Steps(N)` budget, the
two families perform *different amounts* of real env work per
epoch, not just report the same work in different units. At
`n_envs = 32` and typical episode lengths of a few thousand
steps, an on-policy rollout whose episodes all terminate
around step 2000 produces roughly `32 × 2000 = 64{,}000`
`env.step()` calls per epoch; an off-policy inner loop that
runs until the slowest env has completed one episode — say,
step 3000 — performs roughly `32 × 3000 = 96{,}000`
`env.step()` calls per epoch. The disparity is structural: it
comes from the fact that off-policy collects post-termination
transitions to feed the replay buffer, which is what makes it
off-policy, while on-policy has no use for post-termination
data and stops the rollout at `done`. The disparity is not an
accounting error that a counter change can fix, and any
"reconciliation" that merely renormalizes the counters is
hiding a real thing rather than resolving it.

This reframe affects every option the recon session named.

**Option (a) as stated in the recon — "fix off-policy to count
actual steps" — is a reporting-only fix** that would make
off-policy's counter stop incrementing after each env's first
completion. The counter would then produce a smaller number
that looks like on-policy's, but off-policy would still be
stepping the completed envs and still be doing the extra work,
just not reporting it. That is strictly worse than the current
state, because it hides a real disparity behind uniform-looking
numbers. The study rejects this version of option (a).

**Option (a′) — actually change off-policy to stop stepping
envs after they have completed their first episode this epoch
— is a behavior change** that would equalize env work between
the families. It is ~10–20 lines of code in `td3.rs` and
`sac.rs`: track `env_complete[i]` (which the code already does)
and gate the per-env `env.step()` call on `!env_complete[i]`.
But it changes what TD3 and SAC *are*: off-policy algorithms
rely on collecting many transitions per epoch to feed the
replay buffer, including transitions from post-termination
stepping, and cutting that off is not a cosmetic tweak. It is
running TD3 with a changed inner loop and comparing the result
to PPO. Whether that is still "TD3" is an algorithm-design
question the study does not have the standing to settle, and
the rematch cannot run on an altered TD3 without a separate
justification for the alteration. The study rejects this
option as out of scope, while noting it exists.

**Option (b) — document the disparity and live with it** —
accepts that Steps(N) gives the two families the same epoch
count but not the same amount of actual env work, and reports
both families' actual `env.step()` call counts in the rematch
writeup alongside the budget. A reader sees "Steps budget:
16M; on-policy actual env step calls: ~6.4M; off-policy actual
env step calls: ~9.6M" and can form their own judgment about
whether the comparison is fair. The numbers are honest, the
disparity is visible, and the rematch is a comparison of
*reward achieved under a fixed budget* rather than a
comparison of *reward achieved under fixed actual env work*.
The former is what the rematch can currently answer honestly;
the latter is what option (a′) would answer if it were in
scope.

**Option (c) from the recon — introduce a new uniformly-computed
metric — is moot** under this reframe, because both current
counters are already uniformly computing "actual env step
calls made by this algorithm during training." There is no
metric that would produce the same number for both families at
the same epoch count, because the families do different
amounts of work per epoch. A "uniform" metric would either be
(a′) in disguise (by gating off-policy's stepping) or (a) in
disguise (by normalizing the counter post-hoc). Neither
resolves the underlying work disparity.

This chapter picks **option (b) for the rematch** and names
option (a′) as the proper long-term direction for the algorithm
surface, contingent on an algorithm-design discussion about
what "comparable to on-policy" means for an off-policy method
that is designed to collect many transitions per epoch. The
long-term direction is a recommendation, not a dependency;
the rematch can run under option (b) without it.

The risk of option (b) is that the rematch writeup's discipline
has to hold. If the writeup reports only the nominal Steps
budget and omits the per-family actual step counts, the
comparison silently mis-represents the compute each algorithm
used. Chapter 31 inherits this risk and will spec the
writeup-level guard that enforces per-family actual step
reporting. Chapter 22's contribution is to name the failure
mode and the guard in the same place.

## The warmup overhead: document, do not deduct

Chapter 21 established that TD3 and SAC both have a warmup
phase of random exploration that runs before any gradient
update, that the D2c TD3 configuration sets `warmup_steps = 1000`
at `sim/L0/thermostat/tests/d2c_cem_training.rs:321`, that the
warmup is not deducted from the `TrainingBudget::Steps` budget
before the formula runs, and that the resulting overhead is
roughly 32,000 env steps or 0.2 percent of the 16M rematch
budget. The overhead is small in absolute terms and invisible in
the current `total_steps` reporting. The design call is whether
to deduct warmup from the budget, tag it separately in the
reporting, or document it and live with it.

This chapter picks **document and live with it** for the rematch
and **recommends tagging in the reporting** as the long-term
fix.

The case against deducting is direct. Deducting warmup from
the budget means that "TD3 receives `Steps(16M)` minus
`n_envs * warmup_steps` of non-warmup budget" — in other words,
the on-policy algorithms run 100 epochs at the full budget and
TD3 runs something slightly different to equalize *training*
steps. The formula needs a new arm, every call site needs to
know whether it is computing budget for an algorithm with
warmup, and the budget-enforcement guarantee that chapter 21
established as uniform across the algorithm surface becomes
non-uniform. That is a large and cross-cutting change in
service of correcting a 0.2 percent accounting discrepancy, and
the fix costs more than the problem — *conditional on the
rematch's effect size being large relative to 0.2 percent*.
The condition is load-bearing. If the rematch's reward curves
are separated by (say) 3 percent, a 0.2 percent handicap that
favors one family is about a fifteenth of the effect, small
enough that the "live with it" decision is defensible. If the
rematch's curves are separated by (say) 0.5 percent, the 0.2
percent overhead is 40 percent of the effect and the decision
is not defensible — at that point the rematch's headline claim
is within a warmup's distance of zero and the warmup needs to
be either deducted or reported as a separate field before the
rematch can be trusted. Chapter 31's gate is what makes this
conditional explicit: the gate has to bound the rematch's
effect size away from the warmup overhead, and if the gate
cannot be met the decision returns here and the fix stops
being out of scope.

The case for tagging in the reporting is the same as for option
(c) above: a new field, populated uniformly. For algorithms
with warmup, the field's value is `n_envs * warmup_steps`; for
algorithms without, it is zero. `total_training_env_steps` is
then `total_env_step_calls - warmup_env_step_calls`, computed
the same way for every algorithm. A rematch writeup that wants
to distinguish "how many env steps were gradient-guided" from
"how many env steps were random exploration" has the
information.

The case for documenting is the cheapest. The rematch writeup
notes that TD3 and SAC spend the first `n_envs * warmup_steps`
env steps of their budget on random exploration before any
gradient update, that this represents roughly 0.2 percent of
the 16M budget at the D2c configuration, and that a reader
interpreting the reward curves should understand that TD3/SAC
have slightly less effective gradient-guided training than the
on-policy algorithms at the same nominal budget. The reader is
informed; the code does not change; the headline comparison
still holds.

The rematch uses documentation for the same reason it uses
option (b) above: the implementation cost of a better answer is
out of scope for the study, and the rematch is robust against
the small residual error the documentation approach produces.
The writeup makes the overhead visible; a future refactor can
make it uniform across the algorithm surface.

## PPO's `k_passes`: not a budget concern

Chapter 21's short note on PPO's `k_passes` established that the
SGD passes cost wall-clock time but do not advance `total_steps`
and do not affect the budget formula. The design question this
raises for the rematch is whether wall-clock cost should show up
anywhere in the rematch's reporting or whether it should be
ignored entirely.

The answer is that the rematch reports env steps, not wall-clock
time, and PPO's `k_passes` is therefore not part of the
rematch's comparison. A reader who notices that PPO took longer
than CEM in the same number of env steps is seeing a real
phenomenon, but it is a phenomenon about PPO's algorithmic
structure (multi-epoch SGD over a frozen rollout), not about
PPO's use of the env-step budget. The rematch does not compare
algorithms on the structural cost of their learning updates;
it compares them on the reward they achieve under a fixed env
budget. `k_passes` belongs in a performance-engineering
analysis, not in the rematch writeup.

Worth stating the distinction plainly: the rematch answers
"which algorithm learns best per env step," not "which algorithm
should a practitioner choose for a real workload." Wall-clock
belongs to the second question, and a reader who wants the
second question answered is asking something the rematch is not
designed to answer. The rematch is silent on it by construction,
and the silence is a property of the design, not an oversight.

The rematch writeup mentions `k_passes` exactly once, in a
short note that says "PPO runs `k_passes` SGD passes per epoch,
which costs wall-clock time but does not advance the env-step
counter; the rematch does not adjust for this." The note is
there because a reader familiar with PPO will look for it; its
absence would create a question that the rematch should
pre-empt. The note does not change the comparison.

## Summary of design calls

Chapter 22's decisions are:

1. **Budget unit:** env steps, via `TrainingBudget::Steps(16M)`.
   Wall-clock, gradient updates, and sim-seconds are all either
   rejected on uniformity grounds or collapse to env steps
   under fixed hyperparameters.
2. **Rematch configuration:** all five algorithms run with
   identical `n_envs = 32` and `max_episode_steps = 5000`, so
   the Steps-to-epochs formula produces the same 100 epochs
   across the surface. A rematch whose algorithms used
   different parallelism choices would violate the budget-level
   consistency chapter 21 established and is out of scope.
3. **Actual env work per epoch:** the two families genuinely
   do different amounts of env work per epoch. The rematch
   reports per-family actual `env.step()` call counts alongside
   the nominal Steps budget in the writeup, making the
   disparity visible rather than hiding it. The algorithm
   surface could be changed to equalize actual work (option
   a′ in the relevant section), but the change is an algorithm
   design question and is out of scope for the study.
4. **Warmup overhead:** document in the rematch writeup; do not
   deduct from the budget. The decision is conditional on the
   rematch's effect size being large relative to 0.2 percent;
   chapter 31's gate bounds this conditional explicitly.
5. **PPO `k_passes`:** not a rematch concern. Mention once in
   the writeup to pre-empt reader questions. The rematch
   answers "which algorithm learns best per env step," not
   "which algorithm should a practitioner choose" — wall-clock
   is a separate question the rematch is silent on by design.

The rematch can run under these decisions without requiring any
code change to `sim/L0/ml-bridge/src/`. The long-term directions
— option a′ for the inner-loop disparity, warmup tagging in the
reporting — are recommendations the study produces for
execution-phase PRs to consider, not prerequisites for the
rematch.

## What this chapter has not decided

The gated design calls for Part 2 live in chapter 23 (Competition
API v2) and chapter 24 (Result semantics), both flagged for
human review. Chapter 22's decisions are implementation-level
rather than API-level and do not need the same deliberation
layer. The harder design questions — how the five algorithms'
`best_reward` values should be reconciled for cross-algorithm
comparison, what the `Competition` API should look like under a
rematch with replicates — are chapter 23 and chapter 24 work
and are out of scope here.

Chapter 31 (Failure modes) inherits three observations from
this chapter. First: option (b) for the env-work disparity
relies on the rematch writeup being disciplined about reporting
per-family actual step counts, and chapter 31 will spec the
writeup-level guard. Second: the 0.2 percent warmup overhead is
acceptable only if the rematch's effect size is large relative
to it, and chapter 31 will spec the gate that bounds this
conditional explicitly. Third: the rematch's silence on
wall-clock is a property of the design, and chapter 31 may need
to name it in the failure-mode enumeration so a reader who
misuses the rematch as an algorithm-selection guide has a
pre-emptive counter.

Chapter 22's contribution is five design calls for the rematch:
budget unit, rematch configuration, actual-env-work reporting,
warmup handling, and PPO `k_passes` framing. Each call is
implementable against the current `sim/L0/ml-bridge/src/` code
without a refactor. The long-term directions the chapter
recommends — option (a′) for the inner-loop work disparity and
warmup tagging for the reporting — are pointers for execution
PRs, not prerequisites for the rematch. The rematch has what
it needs to run at a defensible level of compute parity, with
the per-family env-work disparity visible rather than hidden.
