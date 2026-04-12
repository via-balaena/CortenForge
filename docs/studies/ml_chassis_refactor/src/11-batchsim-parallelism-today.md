# BatchSim parallelism today

Chapter 10 made a physics argument: per-trajectory noise must be
independent, and the current mutex-guarded shared generator does not
produce independent noise when N environments step in parallel. That
argument rested on a claim about how the current code actually
executes — one shared `Arc<LangevinThermostat>`, one mutex, $N$
environments contending for it. This chapter backs that claim up by
walking the execution trace from the top of `BatchSim::step_all`
down to the line inside the thermostat where the lock is taken. It
is a description chapter, not an argument chapter. The point is to
give the rest of Phase 2 something concrete to point at when it
makes design calls about where state should live instead.

The trace also surfaces a second observation that chapter 13 will
build on: the `BatchSim` test suite contains a bit-exactness test
for parallel vs sequential stepping, but it is scoped to
deterministic physics only, and silently stops covering the
invariant the moment a `LangevinThermostat` is installed on the
model. That gap is not a bug in the test — the test does exactly
what its name says. Chapter 13 is where that observation gets
turned into a finding about latent flakiness. This chapter only
notes that the test exists and that its scope ends at the first
stochastic component.

## The `BatchSim` shape

`BatchSim` lives in `sim/L0/core/src/batch.rs` and is, at its core,
a two-field struct:

```rust
pub struct BatchSim {
    model: Arc<Model>,
    envs: Vec<Data>,
}
```

(Reported at lines 64–67 of `batch.rs`. Every file:line citation in
this chapter is recon-reported against the tree as of 2026-04-12 and
is verified against the current source in the factual pass before
commit, per the protocol in chapter 01.) The shape encodes one piece
of policy and one piece of mechanics. The policy is that `Model` —
the MJCF-derived description of the physics system, including any
passive components installed via `PassiveStack` — is shared across
environments and must therefore be read-only during stepping. The
mechanics is that `Data` — the per-step state of a single simulated
system — lives in its own vector slot per environment, independent
of every other slot.

The constructor `BatchSim::new(model: Arc<Model>, n: usize)` at
lines 73–76 allocates a `Vec<Data>` of length $n$ by calling
`model.make_data()` once per slot. Each `Data` is independently
initialized; there is no cross-environment aliasing. There is also
no seed parameter. The current way to seed a `BatchSim` whose model
holds a `LangevinThermostat` is to seed the thermostat *before*
installing it, at `LangevinThermostat::new(gamma, kT, seed)`, and to
accept that the one seed is the master seed for the one generator
the whole batch will share. The absence of a `new_seeded` variant is
not an oversight the refactor has to add after the fact; it is the
shape the current architecture forces. Once the generator lives
somewhere else — wherever chapter 15 lands — the constructor will
need a seed parameter, and the place where that seed fans out into
per-environment streams will become a genuine design call.

## `step_all` and where rayon enters

The step method is at lines 148–167 and is the fulcrum of the
parallel execution story:

```rust
pub fn step_all(&mut self) -> Vec<Option<StepError>> {
    let model = &self.model;

    #[cfg(feature = "parallel")]
    {
        use rayon::iter::{IntoParallelRefMutIterator, ParallelIterator};
        self.envs
            .par_iter_mut()
            .map(|data| data.step(model).err())
            .collect()
    }

    #[cfg(not(feature = "parallel"))]
    {
        self.envs
            .iter_mut()
            .map(|data| data.step(model).err())
            .collect()
    }
}
```

Two things are worth pinning down. The first is that the parallel
path uses rayon's default thread pool — there is no custom pool, no
configured num_threads, no affinity — so the degree of parallelism
is whatever rayon inferred from the host at process startup. The
second is that the split is per-environment, not per-DOF: rayon
forks one task per env and that task runs that env's entire step
(all passive forces, all constraint solves, all integration) on a
single thread. The physics work for env $i$ never crosses threads
mid-step. From the viewpoint of rayon, `step_all` is an
embarrassingly parallel map over a length-$N$ vector, and in a world
without shared state it would deliver the full speedup.

The `parallel` feature is declared at `sim/L0/core/Cargo.toml:38`
as `parallel = ["dep:rayon"]`, and the `[features]` section at line
37 declares `default = []`. The feature is off by default. A search
of every `Cargo.toml` under `sim/` for a downstream opt-in
(`features = ["parallel"]` on a `sim-core` dependency line) returns
zero matches: no crate in the workspace enables it. Under a plain
`cargo build` or `cargo test` of any L0 or L1 crate, the
`cfg(feature = "parallel")` branch is inert and `step_all` runs
through the sequential `iter_mut()` arm. The parallel branch is
reachable only under an explicit `--features parallel` invocation
or an external consumer that opts in.

This is worth saying plainly because it is one of the facts the
rest of the study returns to. The parallel path exists in the
source, and the `Vec<Data>`-plus-shared-`Arc<Model>` shape is
compatible with a rayon parallel map over envs — that compatibility
is a central reason `BatchSim` takes the shape it does. But
nothing in the workspace turns the feature on today. The sequential
path is what every `cargo test` run actually executes, and the
sequential path has no thread interleaving, which means the
shared-mutex noise problem the next section describes is unreachable
under the default build. The problem becomes reachable the moment
anyone adds `--features parallel` to a real run, which is a
plausible near-future event any time someone measures the training
loop and decides the env map should be run in parallel. Whether
that happens next week or next quarter, chapter 13 treats the gap
as the study's latent-flakiness finding.

The rest of the chapter describes the parallel path, because it is
the path later chapters will weigh fixes against. It is also the
path the architecture is structurally compatible with, which is
the reason the mismatch between "shape says parallel" and "feature
flag says off" is worth flagging rather than ignoring. The scope
of the recon that produced this chapter is the `sim/` workspace
and the `cb_passive` code path; a downstream consumer outside
`sim/` enabling the feature, or an alternate callback installation
site, would invalidate the "no workspace crate opts in" statement,
and a separate verification pass would be needed to cover those.

## The call chain to `PassiveComponent::apply`

Following a single env's task down from `step_all`, the path to the
Langevin lock passes through six stacked calls. Each one is a
function in the deterministic-physics core that has no idea any of
its callees will eventually take a mutex. The chain, with
recon-reported file:line citations:

1. `BatchSim::step_all` at `batch.rs:148` hands each env to
   `data.step(model)`.
2. `Data::step` at `sim/L0/core/src/forward/mod.rs:220` validates
   the input state, routes to the active integrator, and calls
   `self.forward(model)` as the forward-dynamics entry point.
3. `Data::forward` at `forward/mod.rs:279–281` is a thin wrapper
   that calls `self.forward_core(model, true)`.
4. `Data::forward_core` at `forward/mod.rs:408–427` orchestrates
   the forward pass: `forward_pos_vel` first, then `forward_acc`.
5. `Data::forward_acc` at `forward/mod.rs:527–586` does the
   acceleration half of the pass and, at line 535, calls
   `passive::mj_fwd_passive(model, self)`.
6. `mj_fwd_passive` at `forward/passive.rs:348–736` is where the
   passive force accumulators are built. It zeros `qfrc_passive`,
   computes springs, dampers, fluid forces, and gravity
   compensation, and then, at lines 719–721, invokes the stack of
   user-installed passive components:
   ```rust
   if let Some(ref cb) = model.cb_passive {
       (cb.0)(model, data);
   }
   ```

`cb_passive` is a `Fn(&Model, &mut Data)` closure installed on the
model by `PassiveStack::install`. Its body iterates the stack's
registered components and calls each one's `apply` method. For a
`LangevinThermostat`, control eventually reaches
`sim/L0/thermostat/src/langevin.rs:141`, the `apply` implementation.
At line 184 it calls `self.rng.lock()`, and at lines 186–193 it
draws `N_DOF` Gaussian samples from the locked generator and writes
them into the force accumulator.

Everything above the lock runs per-environment on a separate rayon
thread. Everything below the lock runs serialized, one environment
at a time, for as long as the lock is held. The parallel region
contracts to a single sequenced draw, then re-expands. On a 32-env
batch with $M$-step episodes, this contract-and-expand happens once
per env per step — thousands of times per second, with the per-env
draw order determined by whichever thread reaches the lock first
that millisecond. The draw order is therefore a function of thread
scheduling, not of any rule the simulation itself imposes. The
architecture permits that order-variance; chapter 10 is where the
argument that the permission is a physics failure lives, and is
where this chapter hands off.

## The split-borrow and why the signature takes `&Data`

There is a detail inside `PassiveStack::install` that is easy to
misread on a first pass, and it matters for any refactor that wants
to move stochastic state onto `Data`. The closure installed on
`cb_passive` has type `Fn(&Model, &mut Data)` — fully mutable access
to `Data` is available at the call site in `mj_fwd_passive`. But the
`PassiveComponent::apply` trait method takes `(&self, &Model, &Data,
&mut DVector<f64>)` — immutable `Data`, plus a separately-borrowed
mutable `qfrc_passive` buffer. The closure bridges between them by
doing a `mem::replace` trick: it pulls the `qfrc_passive` field out
of `data` into a scratch `DVector`, holds `&data` as a shared
reference to what's left, passes both to each component, and then
swaps the scratch buffer back in after the iteration finishes.

The common misreading is that the `&Data` in the trait signature
reflects a borrow-checker constraint — that `Data` *has* to be
immutable at the component level because something about the way
`cb_passive` is called requires it. It does not. The closure
already has `&mut Data`. The immutable borrow in the trait signature
is a consequence of `PassiveStack`'s current shape, which effects a
partition of `Data` into "the thing you mutate" (`qfrc_passive`)
and "everything else" (`&Data`). That partition matches the
signature of every passive component that doesn't need anything
else from `Data`. It stops fitting the moment one of the components
needs more than `qfrc_passive` — an RNG, say — and the cost of
generalizing it back out is the `mem::replace` machinery being
rewritten to pass `&mut Data` through to the trait method. Whether
the current shape is deliberate design or accumulated path-of-least-
resistance is outside the scope of this chapter; the observation is
only that the shape is what it is, and that it is not fixed by the
borrow checker.

This matters for two reasons. The first is that it removes the
stock defense against moving stochastic state onto `Data`: there is
no "you can't, the trait takes `&Data`" — the trait takes `&Data`
because the callback wrapper chose to hand it `&Data`. The second
is that it opens a branch of the design space the next chapter's
survey will return to. Instead of changing the trait signature, a
refactor could give each environment its own cloned `(Model,
PassiveStack)` pair via a per-env installer, leaving the trait and
the split-borrow untouched and pushing the independence requirement
up to the installer instead of down to `Data`. Chapter 12 will name
that alternative `install_per_env` and weigh it against the
trait-signature change. This chapter only notes that the option
exists and that neither branch is ruled out by the call chain.

## `Data::reset` carries no RNG state today

`Data::reset` lives in `sim/L0/core/src/types/data.rs` in the range
1064–1200. It is the function responsible for restoring a `Data`
instance to an episode-start state: `qpos`, `qvel`, `qacc`,
`qacc_warmstart`, every control channel, every force accumulator
including `qfrc_passive`, `qfrc_spring`, `qfrc_damper`,
`qfrc_gravcomp`, `qfrc_fluid`, and `qfrc_constraint`, plus contacts,
sensors, energy, warnings, sleep state, island state, and mocap
poses. It is comprehensive for the deterministic physics state.

It does not touch any RNG, because there is no RNG field on `Data`
to touch. That is the structural observation this chapter records:
`reset` has no RNG state to preserve, discard, or re-seed, because
no such state exists on `Data` in the current tree. Whether the
refactor should add one, what `reset` should do with it, and how
episode boundaries should interact with per-env seeding are design
calls that chapter 15 will make; this chapter only notes the
current absence so that chapter 15 has a documented starting
point.

## The existing determinism test

The integration test file at
`sim/L0/tests/integration/batch_sim.rs` contains a test named
`batch_matches_sequential_with_contacts` at lines 54–91 that does
exactly what its name suggests: it constructs a four-env `BatchSim`
with a contact-generating model, steps it in parallel, steps an
equivalent sequential arrangement, and asserts bit-exact equality
of the two trajectories. The test passes today. It passes because
the model it uses is deterministic physics only — no
`LangevinThermostat`, no other stochastic component — and under
deterministic physics the parallel and sequential paths produce
identical output regardless of thread scheduling. This is the
correctness invariant the project already cares about enough to
assert.

The test does not cover the invariant under stochastic physics.
The moment a `LangevinThermostat` is installed on the model, the
assertion no longer reliably holds: bit-exactness between the
parallel and sequential paths becomes a function of whether the
two runs happened to see the same thread-scheduling decisions
inside the Langevin lock, which they generally will not. Whether
the test would fail on a given run is itself a flaky property —
it might pass on a four-env four-core machine where the scheduler
consistently picks the same order, and fail on a sixteen-core
machine where contention is high enough to perturb it. No test in
the suite asserts parallel-sequential reproducibility for a
`BatchSim` with a stochastic component installed, and no test
names the flakiness directly. The test that would catch the
regression does not exist yet, and chapter 15 will specify what
it needs to look like.

## What `VecEnv` adds on top

One more observation about where the parallel region fits inside
the broader stepping loop, because it affects which layer the
refactor can even see. `VecEnv::step` at
`sim/L0/ml-bridge/src/vec_env.rs:132–248` is the RL-facing entry
point that drives a `BatchSim` under a training loop. At line 156
it calls `self.batch.step_all()` directly, and the sub-stepping
loop at lines 155–167 has the shape

```rust
for _ in 0..self.sub_steps {
    let errors = self.batch.step_all();
    // ...
}
```

That is, every sub-step is a full `step_all()` call across all
envs, inside a sequential outer loop over sub-steps. The passive
callback — including any Langevin noise draw — fires inside
`step_all()`, which is to say inside the parallel region. The
auto-reset and observation-extraction logic at lines 194–236 runs
sequentially *after* `step_all()` returns. There is no parallelism
above the `BatchSim` layer and no serialization below it; the
thread-fork lives exactly at the `step_all` boundary. The practical
consequence is that any fix for the noise independence problem has
to either change what runs inside `step_all` (by changing where
stochastic state lives, so the call chain no longer hits a shared
mutex) or change what the call chain *sees* at all (by arranging
`BatchSim::new` to install a per-env `PassiveStack` so each env
reaches a different thermostat instance at the leaf). Fixing it
above `VecEnv::step` — for example, by serializing the outer
sub-step loop differently — does not help, because the shared
mutex is reached via a code path that the outer loop doesn't know
exists. Which of the in-scope fixes is the right one is chapter
15's call.

## What this chapter has not argued

The physics reason per-trajectory noise must be independent is
chapter 10's argument, not this one. The enumeration of every
stochastic component in the codebase and the space of faithful
refactor shapes is chapter 12's job. The synthesis that names the
combined effect of "mutex serializes noise" and "no test catches
it" as the study's latent-flakiness finding is chapter 13. The
design decision between putting an RNG on `Data`, installing a
`PassiveStack` per-env, using a splittable key generator, or
generating noise ahead of time is chapters 14 and 15, and both of
those chapters are flagged for human review before commit. This
chapter exists to make those four chapters work: it gives them a
shared concrete picture of how `step_all` executes today, so they
can argue about the fix without having to re-derive the starting
point every time.
