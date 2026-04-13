# Stochastic component survey

Chapter 10 argued from physics that the noise driving each parallel
trajectory must be statistically independent of every other
trajectory's noise. Chapter 11 walked the execution trace from
`BatchSim::step_all` down to the line inside `LangevinThermostat`
where a mutex-guarded generator is drawn from. Both chapters
implicitly assumed that the problem is scoped to one component. This
chapter verifies that assumption by surveying every passive-force
component in `sim/L0/thermostat/src/` and reporting, for each one,
whether it holds stochastic state. The survey also names two things
the surveyed code already contains that later chapters have to
account for: the `Stochastic` sibling trait that gates deterministic
mode, and the `install_per_env` function that opens a second
refactor path alongside "move the RNG onto `Data`."

This chapter is a survey, not an argument. Its job is to answer the
questions chapters 14 and 15 have to answer *before* they can make
design calls: what is in scope, what is not, which invariants must
be preserved across the refactor, and what alternatives are on the
table. The survey's conclusion — that the refactor is scoped to
one file of substantive logic, regardless of which path chapters
14 and 15 pick — is relevant to any cost estimate later chapters
construct, but it is not itself an argument for any particular
fix.

Every file:line citation in this chapter is recon-reported and
goes through the factual pass before commit, per chapter 01. Where
a finding was flagged in the recon as unverified, the factual pass
resolves it against the current source.

## The component inventory

`sim/L0/thermostat/src/` contains six passive-force components, each
implementing `PassiveComponent`. The survey's first finding is that
**exactly one of the six holds stochastic state**:

| Component | File | RNG? | Seed source |
|---|---|---|---|
| `LangevinThermostat` | `langevin.rs` | **yes** (`Mutex<ChaCha8Rng>` field) | `u64` constructor parameter |
| `DoubleWellPotential` | `double_well.rs` | no | — |
| `OscillatingField` | `oscillating_field.rs` | no | — |
| `RatchetPotential` | `ratchet.rs` | no | — |
| `PairwiseCoupling` | `pairwise_coupling.rs` | no | — |
| `ExternalField` | `external_field.rs` | no | — |

The Langevin row's entries are concrete. The struct is declared at
`langevin.rs:72` with a `seed: u64` field at line 75 and an
`rng: Mutex<ChaCha8Rng>` field at line 76. The constructor at line
102 takes `(gamma, k_b_t, seed)` and calls
`ChaCha8Rng::seed_from_u64(seed)` at line 107 to produce the
generator that the mutex wraps. The `apply` implementation at line
141 takes `(&self, &Model, &Data, &mut DVector<f64>)` (the trait
signature), locks the mutex at line 184, draws $N_{\text{DOF}}$
Gaussian samples in the loop at lines 186–193, and writes them into
the force accumulator. The `seed` field is retained alongside the
generator because the `Diagnose` implementation at lines 211–233
formats the seed into a diagnostic summary string; the field is
reachable from diagnostic code but is not read by the step path.

The other five files contain zero matches for `rng`, `Rng`,
`ChaCha`, or `rand::` under grep, and none defines a `&mut self`
method that would let per-component state advance during `apply`.
"Pure" in the survey's sense means "no randomness reached by any
means visible in the current tree and no mutable state advanced
via `&mut self`." This is a lexical-plus-structural claim, not a
semantic proof of purity — a component could in principle become
stochastic in a future change by taking randomness through a
`Data` field it today does not touch — but for the code as it
stands, each of the five produces the same `qfrc_out` contribution
given the same `(&Model, &Data)` pair.

The practical consequence for scope is that the refactor touches
one file of substantive logic, with ripple scope that depends on
which path chapters 14 and 15 pick. If the chosen path changes
the `PassiveComponent::apply` signature, the five pure components
absorb the change as no-op parameter additions: each one ignores
whatever new argument is added. If the chosen path leaves the
trait signature alone and instead gives each env its own
`PassiveStack` (the `install_per_env` alternative discussed
below), no signature change is needed at all and the five pure
components are untouched. Under either path, "trait-wide redesign"
is not on the table; one stochastic component is the full
substantive scope.

## The `Stochastic` sibling trait and its gating test

The survey's second finding is a constraint the refactor has to
preserve across any move of stochastic state. At
`sim/L0/thermostat/src/component.rs:101–108`, the thermostat crate
declares a sibling trait alongside `PassiveComponent`:

```rust
pub trait Stochastic: Send + Sync {
    fn set_stochastic_active(&self, active: bool);
    fn is_stochastic_active(&self) -> bool;
}
```

A stochastic passive component implements *both* `PassiveComponent`
and `Stochastic`. The `Stochastic` trait is the entry point for what
the thermostat crate calls *gating*: the ability to switch a
stochastic component into a deterministic-only mode in which the
`apply` call still runs (to produce any non-stochastic contribution
like Langevin's $-\gamma \dot q$ damping term) but does *not* draw
any noise. The gate is controlled through interior mutability on the
component, so that a caller holding `&LangevinThermostat` can flip
it from shared data without needing `&mut`. This is Decision 7 in
the chassis-level design list and is load-bearing for the tests that
run deterministic physics against stochastic components without
actually being stochastic.

The gating is enforced by a specific test,
`apply_does_not_advance_rng_when_stochastic_inactive`, at
`langevin.rs:312–363`. The test constructs a `LangevinThermostat`,
calls `set_stochastic_active(false)`, runs `apply` twice in a row
against the same `Data`, and asserts that the internal RNG state
has not advanced between the two calls. The test is the mechanical
guarantee that deterministic-mode runs are exactly what they say
they are. It fails, loudly, if `apply` draws noise without checking
the gate first; it fails, equally loudly, if `apply` advances the
RNG state even by one sample in deterministic mode.

This matters for the refactor because the "just move the RNG onto
`Data`" framing does not automatically preserve the test. If RNG
state lives on `Data` — per-env — then the "is the gate on" check
has to work per-env too, which means either the `Stochastic` gate
also moves to `Data` (awkward, because the gate is a
component-level flag, not a per-env state property), or the check
stays on the component and the per-env RNG on `Data` is advanced
only when the component-level gate says it is allowed to advance.
The second shape is workable; neither is a thing the recon framing
"just put the RNG on `Data`" articulates. Chapter 14 has to resolve
it before the refactor can land, and chapter 15 has to write a
test plan that verifies the invariant holds after the move. The
survey's contribution is only to name the constraint early enough
that it is not discovered during the implementation phase.

## The split-borrow in `PassiveStack`, as seen from the survey

Chapter 11 described the `PassiveStack::install` split-borrow in
detail: the callback closure installed on `cb_passive` holds
`(&Model, &mut Data)`, pulls `qfrc_passive` out via
`std::mem::replace`, and calls each component's `apply` method
with `(&Model, &Data, &mut DVector<f64>)`. The survey's concern
with this same machinery is different: not *how* it works but
*what its shape tells us about the trait*. The trait's `&Data`
parameter is a consequence of the callback wrapper's partition
choice, not a consequence of any borrow-checker constraint on
what a passive component could in principle see. The survey
records this fact because it is the structural reason a refactor
that wants to give one component (`LangevinThermostat`) access to
more of `Data` has to touch the wrapper, not just the component:
the wrapper is where the partition lives, and the partition is
what needs rewriting.

The shape is also what licenses the next section's second
alternative. If the problem is that one component per
`PassiveStack` instance shares its state with every env the stack
is installed on, an alternative fix is to have one stack instance
per env, each with its own component instance, each with its own
RNG — without touching the partition at all. That shape is what
`install_per_env` provides.

## `install_per_env` as an alternative refactor path

At `sim/L0/thermostat/src/stack.rs:174–199`, `install_per_env` is
an installation helper that takes an env count $N$ and a
caller-supplied factory of type
`FnMut(usize) -> (Model, Arc<PassiveStack>)`. The helper invokes
the factory once per env, asserts that the returned model's
`cb_passive` slot is empty, installs the returned stack onto the
returned model via the normal `PassiveStack::install` path, and
returns an `EnvBatch { models, stacks }` with $N$ independent
`(Model, Arc<PassiveStack>)` entries. The factory is where the
per-env component construction happens — for a stack containing
a `LangevinThermostat`, the factory's job is to produce a new
`LangevinThermostat` per env, each with its own
`Mutex<ChaCha8Rng>`, and each seeded from whatever per-env seed
derivation the caller implements. The helper's contribution is
not the seeding, which the factory owns, but the installation
plumbing: each returned `Model` has its own `cb_passive` closure
pointing at its own `PassiveStack` instance, and no two envs
share a component instance. The per-component mutex is still
there, but it now guards a per-env generator that no other env
can reach. Contention disappears; independence is recovered; the
trait signature does not change.

This is the alternative to "move RNG onto `Data`" that chapter 11
forward-referenced. The survey's job is to name the alternative
precisely and to note the shape change it forces elsewhere, not
to decide between the two. The shape change is in `BatchSim`
itself: a `BatchSim` today holds `model: Arc<Model>` — one model,
shared across envs. A `BatchSim` under `install_per_env` would
hold $N$ models, one per env, because each env's model carries a
different `cb_passive` closure installed by a different
`PassiveStack` instance. The field turns from `Arc<Model>` into
`Vec<Arc<Model>>` (or some equivalent that keeps per-env models
addressable), the constructor changes to accept a stack factory
rather than an already-installed model, and every caller that
reaches into `self.model` has to pick an env or accept that the
model is no longer batch-scoped.

The cost comparison is genuinely different from the "move RNG onto
`Data`" path and is best summarized as a trade of blast radii.
Each path is characterized below by four axes: structural change,
per-env seed derivation, gating-invariant reconciliation, and
memory cost.

- **Move RNG onto `Data`.**
  - *Structural change:* trait signature grows a `&mut Data` or
    equivalent parameter; the `PassiveStack::install` split-borrow
    wrapper is rewritten to pass `&mut Data` through to the trait
    method; every `PassiveComponent` implementor absorbs the new
    parameter (as no-ops for the five pure components, per the
    smallness finding above).
  - *Per-env seed derivation:* required at `BatchSim::new`, which
    gains a `seed: u64` parameter and fans it out to per-env
    `Data.rng` fields.
  - *Gating reconciliation:* requires chapter 14 to resolve the
    "per-env RNG advance guarded by component-level gate" question
    raised in the previous section.
  - *Memory cost:* one `ChaCha8Rng` per env on `Data` (small).
- **`install_per_env`.**
  - *Structural change:* `BatchSim`'s `model: Arc<Model>` field
    becomes per-env (`Vec<Arc<Model>>` or equivalent); the
    constructor accepts a stack factory instead of a pre-installed
    model; `VecEnv` and any caller that reads `self.model` inside
    the stepping loop adjust to pick an env rather than assuming
    a single shared model.
  - *Per-env seed derivation:* required inside the caller's
    factory closure, which is where each env's
    `LangevinThermostat` is constructed.
  - *Gating reconciliation:* the gate is a per-component flag and
    each env has its own component instance, so cross-env
    reconciliation does not arise; the existing
    `apply_does_not_advance_rng` test still applies to each env's
    instance individually.
  - *Memory cost:* one `Model` per env rather than one shared
    `Arc<Model>`, which may be non-trivial for large MJCFs.

Both bullets exclude test-suite impact, because the same tests
surface under either path: the gating test (`langevin.rs:312–363`)
applies to whichever component instance the refactor produces,
and the parallel-sequential determinism test gap flagged in
chapter 11 needs to be closed by chapter 15 regardless of which
path it picks.

The two paths differ in where the architectural change lives: one
pushes it down into the trait, the other pushes it up into
`BatchSim`. The survey does not pick between them. Chapters 14 and
15 will.

One further clarification is worth recording, because it keeps the
design space from being misread as two-wide when it is actually a
two-by-$k$ grid. Chapter 10 enumerated four faithful shapes for
per-trajectory noise: a stateful per-trajectory RNG, a splittable
key in the SplitMix / JAX `random.key` family, a counter-based
generator keyed by `(trajectory_id, step_index)`, and a
pre-generated noise tape. Chapter 12's two paths — "move RNG onto
`Data`" and `install_per_env` — answer only the question of
*where the state lives*, not the question of *what the state looks
like*. A splittable-key implementation can be hosted on either
`Data` (each env holds its own child key) or on a per-env
`PassiveStack` component (each env's component holds its child
key). A counter-based implementation can skip per-env state
entirely — the "state" is just `(env_index, step_counter)` math —
and is compatible with either hosting choice. The survey reports
on the hosting axis only. Chapter 15 has to pick a point in the
full 2×$k$ grid, and the two axes are genuinely independent:
picking one path on the hosting axis does not constrain the
choice on the mechanism axis, nor vice versa.

One further observation about `install_per_env` is worth recording
here, because it shapes how chapter 14 should read the helper.
The helper is installation plumbing; it does not itself seed
anything. The factory the caller passes in is where per-env
construction happens, and that is where per-env seed derivation
will have to live if the design picks this path. In other words,
`install_per_env` solves the independence-of-component-instances
half of the problem and leaves the independence-of-seeds half
where it has always been: in whoever builds the components. That
is the same "where does the per-env seed come from?" question the
"RNG on `Data`" path has to answer at `BatchSim::new`, just
relocated to a different callsite.

## `GibbsSampler` is not a `PassiveComponent`

There is one more stochastic object in `sim/L0/thermostat/src/`
that the component survey has to address, because a careless read
would count it as a seventh entry in the inventory table above.
`GibbsSampler`, at `sim/L0/thermostat/src/gibbs.rs:26–147`, is a
standalone discrete-spin sampler for Ising-model problems. It owns
its own `ChaCha8Rng` directly — not inside a mutex — and exposes a
`sweep(&mut self)` method that advances the sampler by one Gibbs
step. It is used by reference tests and by standalone sampling
experiments. It does not implement `PassiveComponent`, is not
installed on any `PassiveStack`, and is not reached from
`BatchSim::step_all`. It has nothing to do with the call chain
chapter 11 walked.

The survey mentions it for two reasons. The first is simple
bookkeeping: a grep for `ChaCha8Rng` in the thermostat crate
returns both `langevin.rs` and `gibbs.rs`, and a reader of the
study should not have to re-derive why one is in scope and the
other is not. The second is a one-sentence forward reference:
the implementation asymmetry between Gibbs's per-instance `&mut
self` RNG and Langevin's `Arc`-shared mutex RNG is flagged for
chapter 30's rematch protocol as an experimental-design concern,
so that the rematch's framing does not implicitly treat the two
components as apples-to-apples when the chassis under them is
different. The causal analysis — whether and how much of the
Gibbs-vs-Langevin performance gap is explained by the chassis
asymmetry — belongs to chapter 30, not to the survey.

## What this chapter has not argued

The survey does not choose between "move RNG onto `Data`" and
`install_per_env`. It does not specify how the `Stochastic` gating
invariant is preserved under a per-env state move. It does not
write the regression test that would catch a failed refactor. It
does not decide whether per-env seed derivation should use a
master seed with XOR, a SplitMix64 mixer, `ChaCha8Rng::from_rng`,
or one of the counter-based alternatives chapter 10 enumerated.
All four of those questions are chapter 14 or chapter 15 work,
and both of those chapters are flagged for human review because
the right answer is not obvious from the survey alone.

What the survey establishes, and what later chapters can rely on:
one component holds stochastic state; five components are pure;
the `Stochastic` trait exists and gates deterministic mode; the
`apply_does_not_advance_rng_when_stochastic_inactive` test is the
mechanical guarantee that gating works and is a refactor
constraint; `install_per_env` exists and is an installation helper that
orchestrates per-env construction through a caller-supplied
factory, providing an alternative to the trait-signature change; `GibbsSampler`
is out of scope for the `PassiveComponent` refactor but is worth
mentioning for the rematch framing. That is what the later
chapters take as their starting point.
