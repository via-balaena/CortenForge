# Why this study exists

## The short version

CortenForge's machine-learning infrastructure — `sim-ml-bridge` (the
algorithm framework, policies, training loops) and `sim-thermostat`
(Langevin dynamics, double-well potentials, Ising couplings) — was
built in two short bursts of focused sprint work. Each crate landed
in roughly a day or two. They work. They have been used to run real
experiments and produce real findings. They are not, however, the
foundation we need for years of physics-aware ML research on
thermodynamic computing.

This study is what happens when you stop to inspect the foundation
before pouring another floor of concrete on top of it.

## How we got here

The next planned piece of work is the physics-aware ML pivot:
splitting `sim-ml-bridge` into a chassis crate (`sim-ml-chassis`)
plus an RL baselines crate (`sim-rl`), and adding a new sibling crate
(`sim-opt`) for physics-aware optimization algorithms — Simulated
Annealing first, then Parallel Tempering, CMA-ES, and friends. The
flagship experiment is a rematch of a prior benchmark (D2c) where
standard RL baselines underperformed on a stochastic-resonance task
that physics-aware methods should, in principle, excel at.

A construction spec for that pivot was drafted. Then it was reviewed.
Then it was reviewed again — cold, by someone with no stake in the
previous drafts. The second review's only job was "find every bug,
nit-pick every claim, verify every API citation against source." That
pass turned up the usual mix: wrong line numbers, drifted prose, one
or two misremembered function signatures — the kind of thing a review
is supposed to catch.

And then it turned up three things that weren't nits.

## The three things

**First**, `LangevinThermostat` stores its stochastic state — the
Langevin noise RNG — as a field on the thermostat struct itself,
which is then installed onto a shared `Arc<Model>` that is shared
across all parallel environments in a vectorized simulation. The RNG
is wrapped in a mutex so that concurrent env steps don't race. The
mutex makes the code sound; it does not make the *physics* sound. Two
parallel environments in a `VecEnv` are, by definition, two
independent trajectories. They must see independent noise
realizations, because that is what "independent trajectory" means as
a mathematical object. The current architecture can only give them
independent noise if the runtime happens to schedule their draws in a
repeatable order — which, under rayon, it does not. The fix is
architectural: stochastic state belongs on `Data` (per-trajectory),
not on `Model` (system definition). The current code is a
bug-in-modeling, not a bug-in-implementation.

**Second**, the `Competition` runner that orchestrates
multi-algorithm experiments takes a single seed as input. Every
algorithm in one run gets the same seed. The rematch, as specced,
would compare SA, CEM, PPO, and TD3 at seed 20260412 and report the
winner. This is a known-unreliable experimental protocol — Henderson
et al. (*Deep Reinforcement Learning That Matters*, 2018)
demonstrated that same-algorithm seed-to-seed variance routinely
flips the ranking of two methods. Single-seed comparisons can only
establish effect sizes that are larger than seed variance, and most
of the effect sizes we care about aren't that big. The fix is also
architectural: `Competition` has to accept a vector of seeds and loop
internally, and the result type has to nest per-seed replicates so
mean ± stderr is recoverable.

**Third** — and this one only surfaced when the reviewer was thinking
harder about what "fair comparison" means — Simulated Annealing and
Cross-Entropy Method are specced to run on the same number of
"epochs," but an epoch means wildly different things for the two
algorithms. CEM evaluates `n_envs = 32` candidates per epoch; SA
evaluates 2 (a current and a proposal, paired for a Metropolis
accept/reject step). At 100 epochs, CEM has seen 3200 candidate
policies; SA has seen 200. Depending on how you count "compute," this
is either a 16× handicap in SA's favor (if you count environment
steps, since SA's two evaluations each run across all 32 parallel
envs) or a 16× handicap in CEM's favor (if you count unique
candidates). The fact that the ratio flips depending on which metric
you pick is the real problem: "same epochs" is not a well-defined
comparison between algorithms with different per-epoch cost
structures. The right unit is total environment steps, and the budget
should be specified in those units for any cross-algorithm benchmark.

## Why a study, and not a fix

Any one of the three findings above could be patched in a morning.
The reason to not do that — the reason to stop and write a study
instead — is that these are exactly the kinds of issues you find
when you inspect a foundation. Finding one suggests there are more.
Fixing them one at a time guarantees that the fix for finding #4
won't interact cleanly with the fix for finding #1 because finding
#1 was already shipped.

Physics-aware ML is the research direction. It is going to be years
of work. The infrastructure it runs on has to be defensible,
reproducible, and — critically — *self-describing*, so that someone
picking up the code a year from now does not have to re-derive
everything we learned in this review pass. The study is the record
of that learning. It is longer than a fix PR. It is supposed to be
longer.

## What the study is, concretely

A durable investigation, organized as a book, that walks from the
foundation (how stochastic state is modeled) up through the
algorithm layer (how `Competition` measures things) to the rematch
itself (what we want the first experiment to actually prove). Each
part contains a mix of findings (what is broken and why), design
decisions (how to fix it), and execution plans (what PRs ship in
what order).

Every concrete claim in this book is verified against source on the
way in — see [chapter 01](01-how-this-study-is-produced.md) for the
review protocol — and every design decision is argued explicitly. A
reader who disagrees with a decision can find the argument, find the
alternatives that were considered, and make their own call about
whether the decision still holds. That is the point. The book is
not a set of orders; it is a record of how we thought about this at
a specific moment, with enough context that the thinking can be
revisited.

## What happens after the book

When the study is complete, execution proceeds in three PRs, each
described in detail in Part 4:

1. **Chassis reproducibility** — move stochastic state to `Data`,
   update passive component signatures, seed per-env RNG from
   `BatchSim::new`, establish bit-identical reproducibility as a
   chassis invariant.
2. **Competition replicates and algorithm surface fixes** —
   multi-seed `Competition` API, fair budget units, best-reward
   semantics for replicated runs, fixes to any algorithm whose
   `TrainingBudget::Steps` path turns out to be broken.
3. **sim-opt split and the rematch** — the physics-aware ML pivot
   as originally envisioned, rebased onto the chassis the first two
   PRs produced. This is where Simulated Annealing lands, where the
   SR rematch is run, and where we find out whether the
   physics-aware pivot is vindicated or needs to be rethought.

This is the work plan. Everything before it is groundwork. The
study itself is the most careful piece of that groundwork.

## A note on pacing

This document and the chapters that follow are deliberately slow.
Short-term, the fastest way forward would be to patch the three
findings above and ship the rematch. That is not what is being
done. The reasoning: the cost of a bad foundation scales with how
much is built on top of it, and "years of physics-aware ML research"
is a lot to build on top. Spending a week writing a book is cheap
insurance against spending a year re-deriving what the book would
have recorded. The patience is not an aesthetic preference — it is a
cost-benefit calculation.

If you are reading this book looking for a fast answer, you are in
the wrong document. The fast answer, in summary form, is in the
three PRs listed in Part 4. This book is for the reader who wants
to know *why* those PRs look the way they do.
