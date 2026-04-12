# The wrong struct

The first finding in Part 0 — "`LangevinThermostat` stores its RNG as
a field on a shared struct" — was stated there as a conclusion. This
chapter is where the conclusion is argued from first principles.

The claim, restated in the strongest form the physics alone will
license: **the noise process driving each parallel trajectory must be
statistically independent of every other trajectory's noise process.**
The current code violates this. The violation is masked by a mutex.
The mutex makes the code sound under the Rust type system; it does
not make the physics sound under the definition of a Langevin
trajectory. Those are two different correctness properties, and it
is possible to have one without the other.

The *engineering* question of which struct owns the noise-producing
machinery — `Model`, `Data`, a counter-based generator indexed by
trajectory and step, a pre-generated disjoint tape handed out at
setup — is a separate act of thinking, and the answer depends on
tradeoffs that the physics alone does not resolve. This chapter
argues the physics requirement. The engineering argument is
deferred, explicitly, to chapters 14 and 15.

## What a Langevin trajectory is

The Langevin equation for an overdamped particle in a potential
$V(q)$ at temperature $T$ is

$$\gamma \dot{q}(t) = -\nabla V(q(t)) + \sqrt{2\gamma k_B T} \, \xi(t)$$

where $\xi(t)$ is a Gaussian white-noise process with
$\langle \xi(t) \rangle = 0$ and
$\langle \xi(t) \xi(t') \rangle = \delta(t - t')$. This is the
definition. Everything physical about a Langevin simulation flows
from it. In practice no simulation integrates the continuous SDE; it
integrates a discretized scheme (Euler–Maruyama, BAOAB, and so on)
in which $\xi(t)$ is approximated by a sequence of finite Gaussian
draws, one per step. The physics argument below is really an
argument about the statistical properties of that finite sequence,
which are the things a correct integrator is obligated to reproduce.

The object $\xi(t)$ is a *sample path*. When you ask the Langevin
equation to tell you what a system does, it does not give you one
answer; it gives you a distribution over trajectories, and each
trajectory corresponds to one realization of $\xi(t)$. The ensemble
statistics — mean position, diffusion coefficient, rate of barrier
crossing, the stochastic-resonance peak we care about for the D2c
rematch — are computed by averaging over many independent realizations
of $\xi(t)$, each one producing its own trajectory.

Two trajectories are statistically independent if their noise
sequences $\{\xi_1^{(k)}\}$ and $\{\xi_2^{(k)}\}$ are drawn from
separate, non-overlapping, non-correlated sample spaces. If they
share any of their randomness — if they are drawn from the same
stream, in some order, and that order is itself correlated with
anything in the simulation — then the trajectories are not
independent, and any ensemble average computed from them is biased
by an amount nobody can easily characterize. This is the difference
between "we have N samples from the distribution" and "we have N
samples whose joint law depends on scheduling." The marginals may
be fine; the joint distribution is not, and the joint distribution
is the thing ensemble statistics are computed over.

## What faithful implementations look like

The physics requirement is: each trajectory sees a noise sequence
that is statistically independent of every other trajectory's noise
sequence, and reproducible given the same initial seed. That is a
constraint on the noise *statistics*, not a direct constraint on the
location of any particular struct. Several distinct implementations
satisfy it:

1. **Per-trajectory stateful RNG.** Each trajectory owns a mutable
   RNG whose state advances with each draw. Seeding the RNGs from
   disjoint initial seeds (derived from a master seed by any decent
   mixing function) produces non-overlapping streams under any
   standard generator. This is the most obvious shape and the one
   the rest of this book will adopt, for reasons chapters 14 and 15
   will argue.
2. **Counter-based generator keyed by `(trajectory_id, step_index)`.**
   Generators in the Philox / Threefry / PCG-with-streams family
   are effectively bijections from `(key, counter)` to uniform
   output. A single such generator lives on `Model`, and each
   trajectory reads its $k$-th noise value by asking for the output
   at key `trajectory_id` and counter `step_index`. No per-trajectory
   mutable state exists; the generator is pure. Independence follows
   from the bijection being keyed on `trajectory_id`; reproducibility
   follows from the function being deterministic. This is the
   standard answer in Monte Carlo codes that care about
   reproducibility under parallelism and is not, at all, a
   compromise.
3. **Pre-generated noise tape.** At setup time, generate $N$
   disjoint sequences of Gaussian draws, one per trajectory, sized
   for the full run. At step $k$ on trajectory $i$, read the $k$-th
   value of sequence $i$. No generator runs during the simulation;
   only index math. This is ugly at scale (memory) and inflexible
   (you have to know the run length in advance) but it is
   straightforwardly correct and is used in some high-stakes
   reference implementations.

All three satisfy the physics. The difference between them is an
engineering question about where state lives, what the API surface
is, and how the machinery interacts with the rest of the chassis
(the `Stochastic` gating trait, the `PassiveComponent::apply`
signature, the `BatchSim` environment representation). Chapter 15
makes that call. The point of this chapter is only that **some**
faithful implementation is required, and that **the current code is
not one of them.**

## Why the current code is none of the above

The current implementation is a per-trajectory stateful RNG (shape
1 above) in everything except the critical detail of where the
state lives. `LangevinThermostat` has a `Mutex<ChaCha8Rng>` field —
that is a stateful generator. But the component sits inside a
`PassiveStack` that is cloned via `Arc` into a `cb_passive` closure
installed on a single `Arc<Model>` shared across all $N$ environments
of a `BatchSim`. Every environment reaches the same
`Arc<LangevinThermostat>`, which means every environment shares the
same `Mutex<ChaCha8Rng>`. There is one generator, not $N$, and the
$N$ environments draw from it in whatever order the scheduler
happens to acquire the mutex.

A per-trajectory stateful RNG with $N$ generators is faithful. A
single stateful RNG with a mutex is not. The code is the second
thing while looking structurally like the first; the bug is in the
mismatch.

## Why the mutex is not the fix

The current code guards its RNG with a mutex. In Rust terms this is
correct: it prevents a data race when multiple threads call `apply`
on the same `Arc<Model>` concurrently. In physics terms it is exactly
the wrong fix, because the thing it prevents — a data race — is not
the thing that went wrong. What went wrong is that two trajectories
are sharing one noise process, and the mutex makes that sharing
*orderly* instead of *racy*. Order is not independence.

To see why, imagine two parallel environments stepping in lockstep.
Environment 1 acquires the mutex, draws its noise, releases it.
Environment 2 acquires the mutex, draws its noise, releases it. The
draws are not racy. They are also not independent: environment 2's
noise is whatever the stream produces *after* environment 1 has
consumed its share, and environment 2's noise realization is
therefore a deterministic function of environment 1's. Now introduce
rayon, which does not schedule work in lockstep — the order of
acquisitions depends on thread pool behavior, OS scheduling, cache
state. The correlation between envs becomes non-reproducible as well
as non-independent. Every run draws from the same stream in a
different order and the ensemble statistics are a function of that
order. Bit-identical reproducibility across runs is no longer
attainable for a `BatchSim` containing a `LangevinThermostat`. This
is the latent flakiness that the refactor is attempting to fix: not
a bug that trips a test, but a property that quietly does not hold.

## The smallness of the actual defect

One honest observation before moving on. The architectural fault is
narrow. The survey in chapter 12 will show that only one component
in `sim/L0/thermostat/src/` holds RNG state — `LangevinThermostat` —
and that every other passive component (`DoubleWellPotential`,
`OscillatingField`, `RatchetPotential`, `PairwiseCoupling`,
`ExternalField`) is pure: given the same `Model` and `Data`, they
produce the same force. The wrong-struct problem is a *single
component's problem*, not a trait-wide redesign.

This is good news for the scope of the fix and bad news for the
history of the code, because "only one component is wrong" is
exactly the kind of situation where a morning's patch feels
proportional. The reason we are writing a chapter instead is that the
component in question is the one that the entire physics-aware ML
research direction runs on top of. If Langevin is not reproducible,
nothing built on it is reproducible either, and "nothing built on it"
is, at the moment, every thermo-computing experiment we intend to
run for the next several years. A small defect in a load-bearing
place is worth a large correction.

## What this chapter does not decide

It does not prescribe the fix. At least three shapes are faithful
to the physics (stateful per-trajectory, counter-based, pre-generated
tape), and even within the stateful-per-trajectory family there are
sub-shapes — putting the state on `Data` directly, or giving each
environment its own `(Model, PassiveStack)` pair via a per-env
installer. Each has its own blast radius and its own interaction
with the `Stochastic` gating trait that deterministic-mode tests
rely on. Those are chapter 14 and 15's problem, and both are
flagged for human review because the right answer is not obvious
from the physics alone. The physics only tells us that the noise
sequences must be per-trajectory independent. Which mechanism
produces them is an engineering decision made on top of that
requirement.

For now, the chapter ends on the requirement, not the decision: the
noise driving each trajectory must be that trajectory's own, and
the current code does not supply it.
