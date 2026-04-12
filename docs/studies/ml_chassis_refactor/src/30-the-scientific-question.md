# The scientific question

Chapters 10 and 20 argued that the chassis has to be fixed before
the rematch can be run honestly. This chapter is about what the
rematch is actually asking — what "physics-aware beats generic RL"
means quantitatively, what a clean answer would have to look like,
and what each possible outcome (positive, null, ambiguous) would
tell us. It is the thing the chassis fixes are in service of, and
it exists as its own chapter because "what are we trying to find
out" is easy to lose sight of under the weight of the engineering.

The chapter does not specify hyperparameters, budget, or statistical
gates — those are chapter 23 and 24's problem. This one is about
the shape of the question.

## What stochastic resonance is, as a test problem

Stochastic resonance is the phenomenon where adding noise to a
bistable system at the right temperature *improves* the system's
response to a weak periodic drive, rather than washing it out. It
has real-world instances in climate dynamics, neuroscience, and
mechanical transducers, but for our purposes the important thing is
the shape of its optimization landscape.

In the CortenForge stochastic-resonance task, the agent or optimizer
controls the temperature (via a `kt_mult` multiplier on a base $k_BT$)
and the reward is synchrony between the bistable system's state and
the drive. The prior D2 results established two facts about this
landscape: there is a broad, elevated region of temperature multipliers
where synchrony is above chance — spanning roughly `kt_mult` 1 through
3 — and somewhere inside that region there is a peak. The D2 SR
findings memo records peak synchrony $0.098 \pm 0.022$ near `kt_mult`
$\approx 2.55$ from the `d2b_stochastic_resonance_baselines` temperature
sweep, which places the peak toward the upper shoulder of the elevated
region rather than at its center. The region is wide. The gradient
within it is small. The peak's absolute magnitude is modest; its
statistical significance above the region's floor is not enormous.
All three of these matter for what "solving the problem" looks like.

Concretely: a blind random sampler, given enough budget, can find
the elevated region. Any method whose update is roughly "try a
random parameter, keep it if reward improves" will spend most of
its time somewhere inside it and eventually cluster around the
peak. The hard part is not finding the region. The hard part is
*resolving the peak within it* — reliably ending up near `kt_mult`
$\approx 2.55$ rather than somewhere plausible but worse inside
the elevated region. "Resolving the peak" is what the D2c
experiment was trying to measure. It is also, in its D2c form,
what generic linear-RL failed to do.

## What the D2c result established

The D2c experiment ran four generic RL algorithms — CEM, TD3, PPO,
and SAC — against the stochastic-resonance task with linear function
approximation, 100 epochs, 5000-step episodes, and 32 parallel
environments. The results (recorded in the D2 SR findings memo
and reproduced in the pivot spec's evidence table) were that none
of the four resolved the peak. CEM converged to a point inside
the elevated band but far from the peak (eval `kt_mult` $\approx
0.99$). TD3 and SAC converged to *negative* temperature
multipliers (eval `kt_mult` $= -0.31$ and $-0.78$), a physically
nonsensical result that the findings memo attributes to linear-$Q$
being too weak to represent the value landscape. PPO converged near
zero `kt_mult` but for the wrong reason — a D1d-style
exploration-noise inflation that produced synchrony from transient
exploration rather than from discovered resonance. The memo
summary is: three honest failures and one false positive.

Two observations matter for the rematch. First, the landscape's
broad-and-flat geometry is a structural challenge for any method
whose exploration step size is larger than the in-region gradient —
the method will diffuse across the elevated region without being
pulled toward the peak. Second, the linear approximation floor is
real: the value-based methods (TD3, SAC) failed not because they
couldn't navigate the landscape but because their internal
representation of the value function did not have enough parameters
to track the problem. That second failure is about expressiveness,
not about geometry, and it is the one that a matched-complexity
rematch has to control for.

The geometry-vs-expressiveness distinction is load-bearing for
the rest of this chapter — the rematch tests the first, not the
second — and the distinction only holds if "matched complexity" is
defined operationally rather than rhetorically. If the rematch ends
up running SA with a parameterization that happens to be richer
than the linear-policy class CEM had in D2c, a positive result
could be expressiveness masquerading as geometry and the whole
framing collapses. The operational definition of matched
complexity (parameter count? parameterization family? representation
width?) is a chapter 23 question and has to be nailed down before
any rematch result can be interpreted through the split this
chapter is building on.

## What "physics-aware beats generic RL" is supposed to mean

The rematch adds Simulated Annealing (and, eventually, Parallel
Tempering) to the same pool. The framing of the physics-aware ML
pivot is that these methods are geometry-appropriate for the SR
landscape in a way that CEM is not: SA's Metropolis accept/reject
with a small Gaussian proposal is literally a local random walk
with a temperature schedule, and "local random walk with cooling"
is the standard move for resolving a peak inside a broad band. If
the pivot's strategic claim is correct, SA should be able to do
what CEM could not — not because SA is a more powerful algorithm,
but because its update rule is shaped like the problem.

The operational version of "physics-aware beats generic RL" that
the rematch can actually test is narrower than the strategic
framing, and it is worth being honest about how narrow it is:

- **It is not** "custom methods are better than RL in general."
  That claim would require many tasks, many seeds, and a broader
  evidence base than a single experiment can provide.
- **It is not** "SA beats all RL." TD3 and SAC's D2c failure was
  about linear-$Q$ expressiveness, not geometry, and they will
  fail the rematch for the same reason if they are included. The
  construction spec already excludes SAC from the rematch for this
  reason. **TD3 should be excluded for the same reason.** The
  current construction spec has it in the pool; this is an error
  inherited from the spec's original framing, not a deliberate
  choice. The chapter-20 Henderson argument makes this worse, not
  better: including an algorithm whose D2c failure we already know
  the mechanism of means one of the "algorithms in the comparison"
  is guaranteed to under-perform for a reason unrelated to
  geometry, which drags the "best matched-complexity RL baseline"
  down and makes any SA victory larger in apparent effect size
  than the geometry finding actually supports. Chapter 23 documents
  the decision in the Competition v2 API design; it does not
  re-derive it.
- **It is** "SA, with geometry-appropriate updates and the same
  matched-complexity representation the D2c RL baselines got,
  resolves the SR peak more reliably than those baselines resolved
  it." That is a testable question, the rematch is the test, and
  its answer is informative even though its scope is limited to
  this one task.

The scope limitation is the point. The rematch is a *single* data
point. A single data point is never a proof of a general principle;
it is a proof of concept that the general principle is worth
investigating further. That is the correct ambition for the first
experiment.

## What each outcome would tell us

A careful experiment is designed so that every possible outcome is
informative. The rematch has three meaningful outcomes:

**Positive.** SA resolves the SR peak — converges to a `kt_mult`
near the empirical peak around $2.55$, with reward near the
measured maximum — and the margin over the best matched-complexity
RL baseline is large enough to survive a multi-seed significance
test (chapter 20). The interpretation is narrow and has to stay
narrow: geometry-appropriate updates beat generic RL *on this
particular task, at this particular matched complexity, with this
particular physics-aware method*. It is evidence for the pivot's
strategic claim, not vindication of it — the pivot's strategic
claim is a generalization across many tasks and methods, and one
task with one method can at most be a compatible data point.
Read larger than that, a single positive result would be
exactly the kind of SOTA-by-partition inflation chapter 20 warns
about. This is the outcome the pivot spec hopes for. It is not the
most likely outcome. It would be the cleanest.

**Null.** SA finds the elevated region but, like CEM, cannot
resolve the peak; its best reward is statistically indistinguishable
from the best matched-complexity RL baseline. The interpretation
is: the landscape's broad-and-flat geometry defeats *both*
random-walk SA with a Gaussian proposal *and* generic RL, and the
specific hypothesis that SA's Gaussian-proposal Metropolis step is
geometry-appropriate enough for this landscape is wrong. A null
updates against *that specific hypothesis*, not against physics-
aware ML as a class — but it does put the next experiment
explicitly in charge of carrying the pivot's strategic claim, which
means the claim is on a clock rather than on a concept. The
allowed follow-ups, listed up front so that a "we'll pivot the
experiment" response after the fact doesn't look like motivated
reasoning, are either (a) a richer proposal structure for SA —
non-Gaussian, or CMA-style adaptive covariance — targeting the
same SR task, or (b) Parallel Tempering, which attacks the broad-
region problem with a fundamentally different mechanism, also on
the same SR task. Both are concrete next experiments with
pre-committed test beds; neither is "pick a different benchmark
until one works." Picking a new benchmark post hoc is explicitly
not on this list: if SR turns out to be a bad discriminator for
physics-aware vs generic, that is information about *this
experiment*, not a license to redefine the question after seeing
the answer.

**Ambiguous.** SA beats RL in expectation, but the margin is within
the seed-variance envelope — a plausible false positive from a
favorable seed partition, exactly the kind of result Henderson
warns about. The interpretation is: the experiment as designed
could not discriminate between the pivot's claim and the null, and
what we need is more seeds, tighter effect-size estimates, or a
harder test. An ambiguous result is not a failure of the
experiment; it is a signal to run the next one differently. The
chassis work that chapters 10 and 20 prescribe is exactly what
makes "run the next one differently" a small, cheap operation
rather than a full rewrite.

## What the chapter does not decide

It does not decide the effect size we would call "convincingly
positive." It does not decide the significance threshold. It does
not decide how many seeds constitute enough replicates for this
specific experiment. All of those are chapter 23 / 24 questions,
and the answers depend on measured variance from a small pilot
run that does not yet exist. The one pool-membership call this
chapter *does* make is the linear-$Q$ exclusion: TD3 and SAC are
both out of the rematch pool because the reasoning that excluded
SAC applies identically to TD3, and the chapter's whole geometry-
vs-expressiveness distinction requires it. PPO's D2c failure was
a different mechanism (D1d-style exploration-noise inflation, not
linear-$Q$), so whether PPO stays in the pool is a genuinely
harder call that belongs to chapter 23 — the chapter's reasoning
here does not cover it.

The point of this chapter is only to be clear about what the
rematch is asking and what the possible answers would mean, so
that the chassis work has a target that can be pointed to, and so
that a reader who is skeptical of the whole pivot can see
explicitly what evidence would change their mind — or ours.
