# Latent flakiness

The three chapters before this one each made a different kind of
claim. Chapter 10 made a physics claim: per-trajectory noise must
be statistically independent, and the current shared-mutex
generator does not supply that. Chapter 11 made an execution claim:
here is the exact call chain from `BatchSim::step_all` down to the
line inside `LangevinThermostat` where the mutex is taken, and
here is the test that exists but does not cover the case. Chapter
12 made a scope claim: only one component in the thermostat crate
holds stochastic state, and it is the component on which every
thermo-computing experiment in this codebase rests.

This chapter takes those three claims and names what the combined
shape is. The name is *latent flakiness*, and the word "latent"
is doing specific work. The bug is not hypothetical. It is not
"this would be wrong if we ever did X." It is real, under a
perfectly realizable build configuration, and has been real for
as long as the current implementation has existed. What makes it
latent rather than live is that the specific combination of
conditions required to observe it — `sim-core`'s `parallel`
feature enabled, a `LangevinThermostat` installed on the model,
a workload that exercises the parallel path in `step_all` — is
not a combination that any current workspace crate produces.
Every precondition is in place; no caller has yet flipped the
switch that activates them.

The rest of this chapter defines the term precisely, locates the
finding against the feeder chapters, bounds the severity question
as far as the study can bound it, names the specific failure
mode a future user will experience the first time it goes live,
and sets one concrete requirement for chapter 15: write the test
that would have caught this.

## What "latent" means here

A latent bug in the sense this chapter uses the word is one where:

- The defect is present in code that compiles today.
- A realizable configuration of that code exercises the defect.
- No configuration currently in use by the project exercises it.

All three conditions hold for the shared-mutex noise problem.
Chapter 11 verified the code path against current source at
`sim/L0/core/src/batch.rs:148–167`,
`sim/L0/thermostat/src/langevin.rs:141`, and the intervening call
chain through `forward/passive.rs:719–721` — the defect is in the
tree under the `parallel` feature as of 2026-04-12. The realizable
configuration is `cargo test --features parallel` on any target
that exercises a `BatchSim` holding a `LangevinThermostat`. The
"no current caller" claim is also from chapter 11: the grep over
every `Cargo.toml` under `sim/` for `features = ["parallel"]` on
a `sim-core` dependency line returned zero matches.

The definition above is deliberately neutral about the severity of
the symptom when the configuration goes live. Severity is a
separate axis and is covered in the "how bad is it" section below;
a latent bug in this chapter's sense could in principle be a
panic, a build failure, a correctness regression, or a silent
bias, and the latency framing holds the same way regardless of
which it turns out to be. For this particular defect the symptom
happens to be a correctness failure — non-reproducibility of
results and, per the statistical-dependence argument of chapter
10, a silent bias in ensemble averages — but the word "latent"
is doing work about reachability, not about how much it hurts
when reached.

This is why "latent" is the right word and "hypothetical" is the
wrong one. A hypothetical bug is one that requires a change to
the code before it can matter. A latent bug requires only a
change to a build flag, or a downstream consumer's decision to
opt in. The distinction is the difference between "if we ever
add parallelism" and "the first time we turn on the parallelism
we already have."

## What the feeder chapters establish

The finding rests on four facts from the previous chapters. Two
of them establish that there is a defect. One explains why it
has not bitten yet. One establishes the stakes.

**Defect, leg 1 — physics (chapter 10).** The physics definition
of a Langevin trajectory requires that the noise sequence driving
each trajectory be statistically independent of the noise sequence
driving every other trajectory. Chapter 10 argues this from the
Langevin equation and from the variance formula
$(1/N)(\sigma^2 + (N-1)\bar\rho\sigma^2)$, which shows that when
the average pairwise correlation $\bar\rho$ is nonzero and
unknown, the nominal $1/N$ variance shrinkage of ensemble
averages is not recovered. Every effect-size, error bar, and
significance test downstream of ensemble averaging inherits the
mistake. This defines what counts as "correct" for the system.

**Defect, leg 2 — execution (chapter 11).** The current code
does not satisfy the physics requirement under `--features
parallel`. A single `Arc<LangevinThermostat>` is installed on
one `Arc<Model>` shared by every env in a `BatchSim`, and every
env's `step_all` task reaches through the model's `cb_passive`
callback into the same `Mutex<ChaCha8Rng>`. Under rayon's
`par_iter_mut`, the order in which envs acquire the lock is
thread-scheduled, not seeded, and the draws each env receives
are therefore determined by a source that nothing in the
simulation controls. The chapter 11 draw order is a function of
which thread reaches the lock first in a given millisecond. This
says the current implementation violates the physics requirement
whenever parallel stepping is active.

**Latency (also chapter 11).** The `parallel` feature is
default-off at `sim/L0/core/Cargo.toml:37` (`default = []`) and
no crate in the `sim/` workspace opts in. Under a plain `cargo
test` or a downstream `cargo build` of any L0 or L1 crate, the
`par_iter_mut` branch is inert. The sequential fallback has no
thread interleaving and is bit-reproducible across runs; its
behavior is correct by construction under the physics
requirement, because the question of draw order does not arise
when there is only one thread. The violation exists but is
unreachable from any currently-shipping configuration. Worth
naming explicitly: the sequential fallback's bit-reproducibility
is a gift. It gives the refactor a ready-made reference oracle —
whatever the refactored parallel path produces from a given
master seed, the existing sequential path will produce the same
trajectory, and the assertion a future regression test can make
is therefore "parallel matches sequential" rather than "parallel
matches some pre-recorded tape we hope we got right." Chapter 15
inherits that oracle for free.

**Scope (chapter 12).** The problem is scoped to one component
out of six in `sim/L0/thermostat/src/`, and the component is
`LangevinThermostat`. The other five components are pure and
would not be affected by the problem or its fix. The
`LangevinThermostat` is the one component on which every
thermo-computing experiment this codebase is designed to run
depends. The physics-aware ML research direction is built on
top of it. The defect is narrow in terms of how much code has
to change to fix it, and load-bearing in terms of how much
downstream work depends on it being fixed.

The four facts combine to give the finding's central claim: a
correctness bug is present in the tree under the `parallel`
feature, affects the component that the project's intended
research direction runs on top of, and is unreachable from the
configurations the project currently exercises. The combined
shape is not a theoretical concern about an edge case, and it
is not an active fire. It is a real defect in a dormant state,
waiting for the first configuration change that wakes it up.

## How bad is it when live

One question the feeder chapters do not bound is the size of the
statistical bias $\bar\rho$ introduces once the defect is live.
Chapter 10 argues that $\bar\rho$ is nonzero and unknown; it does
not argue that $\bar\rho$ is large or small. Chapter 13 does not
resolve this either. The failure mode the refactor is guarding
against is not "we know $\bar\rho$ is 0.3 and that is a
show-stopper" but rather "we do not know what $\bar\rho$ is, and
any ensemble average we compute is therefore an estimate whose
bias we cannot characterize." That framing is enough to motivate
the fix, because the point of the ensemble average is to produce
a number that has an error bar attached to it, and an error bar
that does not include the bias is a number that lies.

For a reader who wants a more intuitive sense of the magnitude:
the correlation structure depends on how many noise draws each
env consumes per step and how the thread scheduler happens to
interleave them, and both of those factors change with thread
count, env count, and workload. A configuration that is fine on
one machine could be badly biased on another. That
machine-dependence is itself part of the problem — the bias is
not a fixed quantity the project could empirically measure once
and live with; it is a function of the runtime environment, and
the runtime environment changes every time someone runs the code
on a new machine. The honest answer to "how bad is it" is: we do
not know, we cannot bound it without doing the measurement, and
the measurement is itself a function of a setup that shifts under
us. The refactor is the way we stop needing to ask.

## The symptom class routes debugging away from the physics layer

The specific failure mode matters, because it determines where a
future investigator will look first. When the flakiness goes
live, the observable symptom is non-reproducibility of training
runs — same master seed, different results across runs. That is
the shape of a whole class of RL bugs that have nothing to do
with the physics simulation: unseeded replay buffers,
non-deterministic gradient updates, cudnn mode settings, race
conditions in the optimizer. A researcher seeing
non-reproducibility in a training loop will, correctly and by
training, check those things first. None of those checks leads
to `LangevinThermostat::apply`. The physics simulation is
normally the boring, deterministic bedrock under the noisy RL
algorithm, and it is the last place an experienced researcher
would think to look. This routing of attention is not
hypothetical color; it is a direct consequence of the symptom
class the defect produces, and it is the reason the defect is
worth fixing before anyone has to discover it the hard way.

## The test-shape gap, restated

Chapter 11 described the existing test
`batch_matches_sequential_with_contacts` at
`sim/L0/tests/integration/batch_sim.rs:54–91`. It asserts
bit-exact equality between parallel and sequential stepping for
a four-env `BatchSim` with a contact-generating model. It passes.
It has always passed. It will continue to pass, because the
model it exercises is deterministic physics only and the
assertion it makes is correct for deterministic physics. The
chapter 11 finding was that the test does not cover the case
where a stochastic component is installed on the model.

Chapter 13 turns the chapter 11 finding into a statement about
the *shape* of the test, not the test itself. The test does not
have a bug. The test's scope is narrower than the scope of the
invariant it is nominally checking. The invariant the test
encodes is "parallel stepping and sequential stepping should
produce the same trajectories": that is what the test's name
says, that is what its assertions check, that is what the
project cares about. The scope of the test is "for a model with
contacts and no stochastic components." A reader of the test
suite who wants to know whether parallel-sequential determinism
is maintained across the project's model space has a test that
answers yes for one region of that space and is silent about
every other region.

The silence is the gap. It is not a bug in the test file that
can be fixed by adding an assertion; it is a gap in *what the
test suite covers* that can only be fixed by adding a second
test — one that installs a `LangevinThermostat`, runs the same
parallel-vs-sequential comparison, and asserts the same
bit-exact equality across runs seeded from the same master seed.
That second test does not exist today. When the refactor lands,
it needs to exist, and it needs to exist *before* the refactor
so that the refactor can be judged against a pass/fail
criterion that is not circularly derived from the refactor's
own code. Chapter 15 has to spec this test. Chapter 13's
contribution is to name it as a requirement: a test whose
existence converts the latent flakiness from a defect we reason
about into a defect we verify we have fixed.

Two honest notes about naming. First: the missing coverage is a
defect in the test suite as a whole, even though the existing
test file is not itself defective — the suite is what's
responsible for covering the invariant across the project's
model space, and the suite currently does not. Second: the gap
opened when `LangevinThermostat` was added without revisiting
existing test scope, a natural consequence of adding a new
capability rather than negligence on anyone's part.

## What would turn latent into live

Every one of the following events would flip the flakiness from
latent to live. None is blocked by anything in the current
setup.

1. **A researcher adds `--features parallel` to a `cargo run`
   command line** for a training-loop performance experiment.
   This is a single-action trigger from inside the project and
   the most likely path to live exposure. Rayon's thread pool
   engages, the shared mutex serializes noise draws across
   envs, and the training runs stop being reproducible.
2. **A downstream crate adds
   `sim-core = { features = ["parallel"] }` to its own
   `Cargo.toml`.** This is a single-action trigger but it
   propagates to every user of that downstream crate, which may
   be more than one researcher and may involve people outside
   the immediate study context.
3. **A CI job is added that runs the existing integration tests
   under `--features parallel` as a matrix cell.** The
   `batch_matches_sequential_with_contacts` test will still pass
   (it is scoped to deterministic physics). Any new test written
   against a stochastic model will expose the bug, but because
   the new test will land together with the refactor, the CI
   matrix cell is not a discovery pathway so much as a
   verification pathway for the fix that lands alongside it.
4. **An external user — a research group consuming CortenForge
   as a dependency — enables the feature in their own consumer
   crate.** This trigger involves a non-project actor and is
   the only path that does not route through someone who is
   aware this study exists. It is the least-protected case and
   the one where the debugging time will be longest.

Events 1 and 2 are the realistic near-term triggers. Event 3 is
conditional on the refactor itself and is more of a verification
pathway than a discovery pathway. Event 4 is the long-tail
exposure that is hardest to protect against. The sequence of
causes that produces a live failure in the near term is short,
and the window closes the first time any of events 1, 2, or 4
happens.

## What this finding says about urgency

The chapter has now said everything the physics, the execution
trace, and the scope analysis can say without making a decision
about the fix. The decision itself is chapter 14 and chapter 15
work. But chapter 13 has a responsibility to say what the
finding implies for how urgent that work is, because "latent"
is not the same as "not urgent" and a reader could come away
with the wrong read.

A defect that (a) is present in the tree under the `parallel`
feature, (b) affects the component the project's research
direction runs on, (c) becomes observable the moment a single
feature flag is flipped, and (d) presents as the kind of bug
that gets mis-attributed to the layer above is urgent in a
specific sense. It is not urgent in the sense of "something is
on fire right now." It is urgent in the sense that the cost of
delaying the fix grows with each new workspace caller and has
no natural ceiling before the first live failure. The cost
model is not "a fixed amount of debugging time per latent day";
it is "a step function that jumps the day the flag flips, plus
a tail that grows with each caller who has to rediscover the
problem independently after that."

The window for fixing the defect without anyone seeing it live
closes the first time a workspace caller — or a downstream
consumer — enables `--features parallel` on a `BatchSim` holding
a stochastic component. That is a property of the problem, not
a directive about how the fix should be designed; chapters 14
and 15 will decide the shape of the fix on their own terms.
Chapter 13's contribution to the urgency question is only to
name the window as real and the clock as running.

## What this chapter has not argued

The fix is not here. Chapter 14 picks between "move RNG onto
`Data`" and `install_per_env`, and chapter 15 makes the design
calls (per-env seed derivation, gating preservation, test
specification). Both are flagged for human review because the
right answer is not obvious from the analysis so far.

The test specification is also not here. Chapter 13 names the
test as a requirement — "a parallel-vs-sequential determinism
test for a `BatchSim` with a stochastic component installed" —
but does not write the assertion list, pick the master seed
count, decide whether it is an integration test or a bench-level
test, or choose a tolerance. Those are chapter 15's calls.

The communication plan for downstream consumers — whether the
project should send a note to known users enabling `--features
parallel`, whether the refactor should be announced in release
notes, whether a feature-gate warning should be added as a
temporary guard — is out of scope for the study. These are
project-management decisions that depend on context the study
is not the right place for.

What chapter 13 does contribute: the name *latent flakiness*,
a precise three-condition definition of what makes a bug latent
in this sense, the four facts from chapters 10, 11, and 12 that
establish the defect and its stakes, an honest bound on how
badly it hurts when live (we do not know, and the measurement
is machine-dependent), the symptom class that routes future
debugging away from the physics layer, the test-shape gap
restated as a requirement for chapter 15, the enabling events
that would flip the state, and the urgency framing that names
the window as real and the clock as running. That is the
output.
