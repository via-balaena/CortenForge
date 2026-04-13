# The new shape

This chapter is the first of two that take the problem Part 1 has
established and turn it into a design. Chapter 10 argued that the
noise driving each parallel trajectory must be per-trajectory
independent. Chapter 13 argued that the current code's violation of
this requirement is latent but real — unreachable under default
feature flags today, reachable the moment anyone enables `--features
parallel`. Between those two chapters the book has named what is
wrong and why the wrongness matters. It has not yet named what the
fix looks like.

The fix has two axes. The first is *how* the noise process is
produced — what mathematical object generates the per-trajectory
draws. Chapter 10 enumerated four such mechanisms (per-trajectory
stateful RNG, splittable keys, counter-based PRF, pre-generated
tape) and said all four satisfy the physics requirement. The second
is *where* the state associated with that mechanism lives in the
chassis — on the `Data` struct, on a sibling struct owned by
`BatchSim`, or on per-env component instances installed via the
`install_per_env` machinery that already exists in the thermostat
crate. These two axes are independent. The mechanism question is
about mathematics; the hosting question is about chassis boundaries.
Any mechanism can compose with any hosting choice and the physics
still works; the engineering tradeoffs differ.

Chapter 14 lays the two axes out. It walks the four mechanisms once,
walks the three hosting choices once, then composes them in a short
reference grid showing which of the resulting cells are live
candidates, which collapse into each other, and which are eliminated
for reasons named in the walk. It does not pick a cell. Chapter 15
picks, after a thinking pass against this chapter's grid.

The reason for the two-chapter split is that the mechanism axis and
the hosting axis each carry their own costs, their own tradeoffs
with the rest of the codebase, and their own interactions with the
constraints inherited from Part 1 and Part 2. Walking the grid
cell-by-cell would force each cell to re-describe the same mechanism
cost or the same hosting cost, and the chapter would be long in
proportion to the number of cells rather than the number of distinct
concepts. Axes-first also lets a reader who cares only about the
mechanism question read section 1 and stop; a reader who cares only
about hosting can read section 2 and stop. The grid is navigation.

## What Ch 14 inherits from Part 1 and Part 2

Before walking the axes, the chapter names the constraints that any
cell must satisfy. These are not decisions Ch 14 makes; they are
conclusions from earlier chapters that Ch 14 treats as given.

**The physics constraint (Ch 10).** Each trajectory sees a noise
sequence that is statistically independent of every other
trajectory's noise sequence, and reproducible given the same initial
seed. Chapter 10 enumerated four mechanisms that satisfy this
constraint; any fifth mechanism Ch 14 considered would have to be
argued for against that enumeration. Ch 14 does not revisit whether
a given mechanism is physics-faithful. It takes the enumeration as
closed.

**The gating constraint (Ch 12).** The `Stochastic` sibling trait at
`sim/L0/thermostat/src/component.rs:101-108` (recon-reported) is
load-bearing for finite-difference and autograd contexts. When a
stochastic component's `set_stochastic_active(false)` is called,
`apply` must produce its deterministic contribution (e.g.,
Langevin's `-γ·v` damping) without advancing any RNG state. The
invariant is operationalized by
`apply_does_not_advance_rng_when_stochastic_inactive` at
`sim/L0/thermostat/src/langevin.rs:311-363` (recon-reported): a test
that sets inactive, runs two applies, flips to active, constructs a
fresh thermostat with the same seed, and asserts the two first
noise samples are equal. This has to keep working under any
refactor. All four mechanisms in Ch 10's enumeration satisfy this
constraint — the entry points differ (a flag early-return, a
"don't consume the key this step," a "don't increment the counter
this step," a "don't advance the tape read index this step") but
each mechanism has a natural gating shape. The trait is not a
filter on mechanisms; it is a uniform requirement that every
candidate must respect.

**The oracle (Ch 13).** The sequential fallback — the `!parallel`
branch of `BatchSim::step_all` — is bit-reproducible today and stays
bit-reproducible under any refactor that keeps the sequential branch
deterministic. Ch 13 named it as a ready-made regression-test
oracle: any future test can assert "parallel matches sequential for
the same master seed" rather than needing a pre-recorded noise tape
or an external reference implementation. Ch 14 does not make this a
selection criterion, because every candidate preserves the
sequential branch's determinism; but Ch 14 is honest that this
oracle is why the "parallel and sequential should agree" assertion
becomes testable in a way that was not available to the current
code's `batch_matches_sequential_with_contacts` test.

**The `BatchSim` shape (Ch 11).** `BatchSim` is a two-field struct
at `sim/L0/core/src/batch.rs:64-67` (recon-reported): `model:
Arc<Model>`, `envs: Vec<Data>`. All environments share a single
`Arc<Model>`; each environment has its own `Data`.
`BatchSim::new(model: Arc<Model>, n: usize)` at `batch.rs:73-76`
(recon-reported) allocates the `envs` vector by calling
`model.make_data()` `n` times. There is no seed parameter today.
Any cell that changes where stochastic state lives may or may not
also change this constructor's signature; Ch 14 notes the
implication under each hosting choice.

## Section 1 — The mechanism axis

### Shape 1: per-trajectory stateful RNG

The mechanism the current code already uses, minus the shared-state
defect. Each trajectory owns a mutable generator whose state advances
with each draw. Seeding the generators from
disjoint initial seeds — derived from a master seed by any standard
mixing function such as splitmix64 — produces non-overlapping
streams under any of the usual generators (ChaCha, PCG, Xoroshiro).
Reproducibility follows from the generator being deterministic
given its seed; independence follows from the seeds being disjoint.

This is the shape the current code already has, in everything
except where the state lives. `LangevinThermostat` at
`sim/L0/thermostat/src/langevin.rs:76-77` (recon-reported) declares:

```rust
rng: Mutex<ChaCha8Rng>,
stochastic_active: AtomicBool,
```

The generator is a ChaCha8 stream; the mutex is there because
`PassiveComponent` requires `Send + Sync` and `ChaCha8Rng` is not
itself `Sync`. The seed is consumed at construction time
(`langevin.rs:107`, recon-reported) and the stream advances with
each `apply` call.

What shape 1 needs from the chassis, described independently of
hosting: one mutable generator instance per trajectory, each holding
its own stream state. The trait signature does not need to change —
`apply` already takes `&self` plus `&Model`, `&Data`, and the
output buffer, and shape 1 works inside that signature as long as
interior mutability on the component is acceptable (which the
current code already uses). Gating works via the same flag pattern
the current code uses: the flag is read, and on inactive the `apply`
method early-returns *before* touching the generator, so no stream
advance happens. The existing test at `langevin.rs:311-363`
operationalizes this invariant exactly.

Cost: one generator's worth of state per trajectory. For ChaCha8,
this is on the order of a hundred bytes per env, times `n_envs`
(typically 32 in the rematch configuration). Negligible.
Dependencies: `rand_chacha` is already in the workspace. No new
crate.

What shape 1 *does* impose structurally is interior mutability on
the component — specifically, a mutex wrapping a non-Sync stream
generator, or some other wrapper that satisfies `Sync`. The current
mutex is the natural fit; alternatives include `RwLock` (no
advantage, the access pattern is always write-exclusive) or
`Cell<Option<ChaCha8Rng>>`-based patterns (not `Sync`). Whatever the
wrapper, a reader of `langevin.rs` under shape 1 has to understand
three things to understand the RNG field: why is this
interior-mutable, why is this `Sync`-wrapped, and why is this
uncontended in the happy path. These are separate questions from
the physics; they are load-bearing for the Rust type system.

### Shape 2: splittable keys

Generators in the SplitMix / JAX `random.key` family support a
`split` operation that takes a parent key and produces `n` child
keys guaranteed to produce non-overlapping sequences. A single
parent key lives on the simulation setup (plausibly on `Model`, or
on a master seed struct); each trajectory receives its own child
key at init; child keys advance per-trajectory without coordination.
This is the functional-programming answer and is what
reproducibility-first ML codebases (JAX, Equinox) reach for when a
pure-function interface matters.

In a language with a standard splittable RNG, shape 2 is a distinct
mechanism with a different API surface from shape 1. A draw takes a
key and returns `(key, value)` — the caller is responsible for
threading the new key forward. The generator is pure.

In Rust, shape 2 collapses in two directions. A stateful-child-
from-stateful-parent reading of "splittable" collapses into shape 1:
the `rand` ecosystem has `SeedableRng::from_rng(parent)`, which seeds
a child from a parent's next bytes, but this advances the parent's
state (a mutable operation) and the child is itself a stateful RNG
with a mutable stream. The "pure functional key" property is not
preserved; what you get instead is a pattern that looks like shape
1 with a different initialization API. A pure-key-on-PRF reading of
"splittable" — the reading a codebase committed to the functional
shape would actually want — collapses into shape 3: a custom
splittable RNG implemented on top of a PRF is counter-based
mechanics dressed in a splittable-key interface. Either way the
axis has three live entries, not four.

Ch 14 lists shape 2 separately for honest coverage of Ch 10's
enumeration and for readers coming from a JAX / functional
background. Subsequent sections do not give shape 2 its own
hosting analysis; any hosting choice for shape 1 applies to a
Rust-flavored shape 2 identically.

### Shape 3: counter-based PRF

Pseudorandom functions in the Philox / Threefry / PCG-with-streams
family are effectively bijections from `(key, counter)` to uniform
output. A single key lives on the simulation setup; each trajectory
reads its $k$-th noise value by asking for the output at key
`trajectory_id` and counter `step_index` (with DOF index folded in
as an inner counter). No per-trajectory mutable RNG stream exists
at all; the generator is pure.

The state shape 3 needs is not a stream. It is two per-trajectory
integers plus a master seed shared across the batch. A `traj_id:
u64` that is constant per trajectory, a `step_index: u64` that
advances by one per step, and a `master_seed: u64` that lives on
whichever chassis struct the hosting axis ends up placing it on
(`Model` is the natural home under hostings B and C since the
master seed is part of the simulation's reproducibility contract
rather than per-env state; hosting A has its own question about
where the master seed goes, addressed in Section 2). The noise at
step $k$ on trajectory $i$ for DOF $d$ is `PRF(master_seed,
traj_id=i, step_idx=k, dof=d)` — a pure function of four integers.

Shape 3 changes the character of the component's `apply`
implementation more than any other shape on the axis. Under shape
1, `apply` reads from a stream; under shape 3, `apply` computes a
hash-like function from known integers. The component's only
remaining state is the counter itself plus whatever constant fields
(like `traj_id` and the DOF count) the component was constructed
with. There is no `Mutex<Stream>`; there is no "interior mutability
on a non-Sync generator." If the counter is stored in an
`AtomicU64`, the field is `Sync` natively and lock-free, and
`fetch_add` replaces `lock`.

Gating under shape 3 is a single conditional: if the flag is
inactive, do not advance the counter this step. The Phase 5 FD
invariant — two applies under `disable_stochastic` must produce
identical deterministic forces, with no intervening RNG advance —
is satisfied by the same early-return pattern the current code uses,
just with the integer counter standing in for the ChaCha stream
state.

The cost shape 3 imposes is a PRF implementation. Three routes
are available in Rust, each with its own cost profile.

**Route 1: use `rand_chacha`'s existing seek API.** `rand_chacha
0.9.0` (already in the workspace) exposes
`ChaCha8Rng::set_word_pos(u128)` as a public method that seeks to
an arbitrary position in the underlying cipher stream. Each apply
call can seek to an encoded `(traj_id, step_index, dof_idx)`
position and read one value, giving counter-based PRF semantics
on top of the chassis PRNG with no new dependency. The cost is
that `ChaCha8Rng` is not `Sync`, so the component still needs
interior mutability (the mutex or equivalent), and the
`set_word_pos` call itself costs roughly one ChaCha block
computation per seek. A reader under this route sees
`Mutex<ChaCha8Rng>` on the component and has to learn both the
mutex-ceremony questions from shape 1 *and* the seek-based
counter semantics — arguably more complicated than shape 1, not
less.

**Route 2: implement the ChaCha8 block function directly in the
thermostat crate.** Roughly 50 to 100 lines of well-documented
block cipher code, as a stateless pure function verifiable against
`rand_chacha`'s output for correctness, and free of
interior-mutability ceremony on the component because no RNG state
is stored as a field. A reader sees `apply` calling a pure
function and understands it in one read. The cost is that the
thermostat crate owns a small cryptographic primitive that has to
be maintained, tested, and kept consistent with the specification.

**Route 3: depend on a counter-based PRF crate.** The Rust
ecosystem does not have `rand_philox` or `rand_threefry` in active
maintenance (both names return empty or abandoned results on
crates.io); the candidates that do exist are `aprender-rand`
(Philox 4x32-10 implementation, version 0.29.0, from the
`paiml/trueno` repository) and `squares` (a smaller counter-RNG
crate at version 0.1.1). Both are real and usable but neither is
as widely adopted as `rand_chacha`. Taking on either amounts to
depending on a less-battle-tested crate for a load-bearing piece
of the chassis. For a production or security context the risk is
real; for a research codebase whose PRF output is fed through
Gaussian transforms and consumed by an SDE integrator, a
defective PRF would surface as bias in the rematch's measured
statistics — which the regression-test oracle from Ch 13 would
catch before it reaches published numbers.

Ch 14 does not pick among these routes — that is Ch 15's
decision — but names them honestly so the grid in Section 3
treats shape 3's cost as an implementation cost (one of the three
routes above), not an unavoidable dependency on a niche crate.

What shape 3 needs from the chassis, described independently of
hosting: a way for `apply` to read the per-trajectory `traj_id` and
the per-step `step_index` at call time. Three places these integers
can live are named under the hosting axis in section 2; which place
is right is a cell-selection question deferred to Ch 15. The
important thing for the mechanism axis is that shape 3 requires the
chassis to surface these two integers to `apply`, and the chassis
today does not surface them anywhere.

### Shape 4: pre-generated noise tape

At setup time, generate `n_envs` disjoint sequences of Gaussian
draws, one per trajectory, sized for the full run. At step $k$ on
trajectory $i$, read the $k$-th value of sequence $i$. No generator
runs during the simulation; only index math. This is shape 4 from
Ch 10's enumeration and is used in some high-stakes reference
implementations where the property "the noise is inspectable and
reproducible by anyone with the tape" is worth the cost.

The cost is the cost. The rematch budget is 16M env steps in
aggregate across the batch (Ch 22's Steps-as-total-env-steps
convention, not 16M steps per env), with `n_envs=32` so each env
runs 500K steps across the full training budget. A Langevin tape
sized for the full run requires `n_envs * steps_per_env * n_dofs *
8` bytes of pre-generated noise. At one DOF and 8 bytes per draw
that is 4 MB per env and 128 MB total across the batch. At two DOFs
it is 256 MB. At the six-DOF SR particle of the D2c rematch it is
768 MB. These are not prohibitive numbers for a modern workstation,
but they are noticeably non-zero, and the memory scales linearly in
every dimension that might grow (more DOFs, more envs, more
steps). Under the alternate reading "16M steps per env," the numbers
are 32× larger and shape 4's elimination is even more decisive;
the chapter uses the Ch 22 convention throughout.

More problematic than the memory is the inflexibility. Shape 4
requires knowing the run length in advance. A `BatchSim` whose max
episode length is known at construction can allocate the tape
statically; a training loop that reuses envs across episodes of
varying length has to re-derive the tape index each episode, which
is a different kind of counter state than the one shape 3 requires
and does not get the "no mutable state" property back. Reset
semantics for shape 4 are also their own question — does a reset
draw from the same tape region, from a fresh region, or does each
episode get its own tape?

Shape 4's one virtue is that it is straightforwardly correct and
the noise is inspectable. If the rematch ever wants a property like
"two independent runs should see the same noise at the same (traj,
step) indices," shape 4 makes that trivially provable by comparing
tapes. Shape 3 also has this property through the PRF's
determinism; shape 4 has it through explicit storage.

Ch 14 lists shape 4 for honest coverage of Ch 10's enumeration.
Under every hosting choice in section 2, shape 4 imposes memory
costs that the other shapes do not, and the memory scales with run
length in a way that interacts poorly with the open-ended training
budgets the rematch depends on. Section 3's grid eliminates shape
4 cells on these grounds; Ch 14 names the elimination now so
readers do not expect the Ch 15 shortlist to treat all four shapes
as co-equal.

### Summary of the mechanism axis

Shape 1 is the current code's shape, rewritten to own its stream
properly. Shape 2 collapses (in two directions) into shapes 1 and
3 in Rust. Shape 3 is a genuine second mechanism with different
chassis demands and a PRF implementation cost (one of three
routes named above). Shape 4 is an also-ran with real memory and
flexibility costs that section 3's grid eliminates.

The mechanism axis has three live entries under the rest of this
chapter: shape 1, shape 3, and shape 4 (with shape 4 kept for
completeness and eliminated in the grid).

## Section 2 — The hosting axis

The hosting axis is the question: given a mechanism, where in the
chassis does its state live? Different mechanisms need different
kinds of state — a stream for shape 1, two integers for shape 3, a
tape and a read index for shape 4 — but the hosting question is
structurally the same for all of them: what struct holds the state,
how does the state get to `apply`, and which chassis boundaries are
crossed in the process.

Three choices are visible in the chassis as it stands today. The
chapter walks each independently. A fourth candidate — putting
per-env state on `Model` as a `Vec` indexed by env id — collapses
into hosting B (it is structurally a sibling vector to `envs`, just
owned by `Model` instead of `BatchSim`) and is additionally a worse
fit because `Arc<Model>` sharing is designed for read-only access
during stepping. Other candidates (thread-local, static global,
the `Data::plugin_state` escape hatch) are named in passing through
the walk below where relevant and dismissed for specific reasons.

### Hosting A: on `Data` directly

The `Data` struct at `sim/L0/core/src/types/data.rs:34`
(recon-reported) is the dynamic simulation state — one instance per
environment, allocated by `Model::make_data()`, passed into `apply`
as the `&Data` argument. Every environment already has its own
`Data`, so "per-trajectory state" is structurally already there;
the question is whether the state associated with the
stochastic-noise mechanism should live as a new field (or fields)
on `Data`.

Hosting A under shape 1 would mean a new field like `langevin_rng:
Option<ChaCha8Rng>` on `Data`, mutable, accessed by the Langevin
component during `apply`. This has two immediate problems. First,
the field is component-specific — it couples `Data` to the Langevin
thermostat's internals, which is the opposite of the
`PassiveComponent`-as-aftermarket framing. Second, `apply` currently
takes `&Data` (immutable); mutating a field on `Data` from `apply`
requires either changing the trait to `&mut Data` (a chassis-wide
breaking change that touches every existing `PassiveComponent`
impl) or putting the field behind interior mutability on `Data`
itself. The first is a wider refactor than almost any other option
in the grid; the second grows `Data` with a
`Mutex<Option<ChaCha8Rng>>` that reintroduces the very
mutex-as-ceremony cost shape 1 already imposes, just in a different
struct. Cell A-1 stands out from B-1 and C-1 in a specific way:
B-1 takes one cost (trait signature change) in exchange for the
mechanism's benefit; C-1 takes one cost (`BatchSim::new` rewire)
likewise; A-1 takes *both* costs at once (trait change and
mutex-in-a-different-struct) for no additional benefit. The grid
in Section 3 carries A-1 through to explicit elimination rather
than concealing it here, but the "no additional benefit over paying
both costs" observation is what the elimination will rest on.

Hosting A under shape 3 is more interesting and is the cell the
`Data` invariant question attaches to. Shape 3 needs `step_index:
u64` and `traj_id: u64` reachable from `apply`. Placing them as new
fields on `Data` means two `u64` fields, both `Copy`, both trivially
incorporated into the manual `Clone` impl at `data.rs:671`
(recon-reported), both readable from `apply` without any trait
signature change. The mechanical cost is near-zero.

The contested question is the documented invariant at
`data.rs:27-30` (recon-reported):

> **Key Invariant:** `qpos` and `qvel` are the ONLY state variables.
> Everything else (xpos, xquat, qfrc_\*, etc.) is COMPUTED from them
> via forward dynamics.

A strict reading of this invariant disqualifies shape 3 on `Data`:
`step_index` is not `qpos` or `qvel`, is not computed from them via
forward dynamics, is not an input (like `ctrl` or `mocap_pos`), and
does not fit the "physics state or physics-input" scope the
invariant implies. Under the strict reading, adding `step_index` to
`Data` would be smearing RL-discretization bookkeeping into the
physics chassis, and the whole point of the refactor is to respect
chassis boundaries.

A loose reading of the same invariant reaches a different
conclusion, though the reading is suggestive rather than
dispositive. `Data` already has `time: f64` at `data.rs:520`
(recon-reported) — a scalar that advances each step and is neither
`qpos`/`qvel` nor computed from them via forward dynamics. `time`
is on `Data` because the physics needs to know the independent
variable of the SDE, and components like `OscillatingField` and
`RatchetPotential` read it as a physics quantity. `step_index`
would be a *bookkeeping* sibling — a per-step integer counter used
only as a PRF index, with no physics use. The two are related
arithmetically (`step_index * timestep = time` under uniform
stepping) but play different roles: `time` is a physics quantity
that happens to also advance per step, `step_index` is a pure
bookkeeping quantity. The precedent `time` sets is not "`Data`
tolerates arbitrary counter fields" but something weaker:
"`Data` tolerates a scalar clock field because the physics needs
one." A strict reader can accept `time`'s presence and still
reject `step_index` on the grounds that the physics does not need
it. A loose reader can argue that once `Data` has one clock field,
a sibling clock field in integer units is a small extension.
Neither reading is obviously correct; the question is genuinely
contested.

A second piece of evidence points the same way: `Data` at
`data.rs:606` (recon-reported) already has a CortenForge-specific
extension, `qDeriv_pos`, documented as "CortenForge extension —
MuJoCo has no equivalent." CortenForge has already grown `Data`
beyond the mjData template. The invariant is leaky in practice. A
field like `step_index` would be a new category of leak — not a
physics extension like `qDeriv_pos`, but a discretization-
bookkeeping field — and the question is whether the chassis
tolerates that category or confines it elsewhere.

`traj_id` is the more awkward of the two. `traj_id` has no physics
meaning — it is pure RL-bookkeeping metadata used only by the PRF
to keep trajectory streams independent. A strict reading of the
invariant disqualifies `traj_id` on `Data` even more firmly than
`step_index`. However, under hosting choice C (per-env component
instance), `traj_id` can live on the component instance itself as a
`const` field set at construction time, never on `Data` at all; this
is worth flagging here so the grid in section 3 does not
double-count `traj_id` as a hosting-A-specific cost.

Hosting A under shape 4 would mean a per-env tape and a tape-read
index as new fields on `Data`. The tape is large (see shape 4's
section). The read index has the same status as shape 3's
`step_index`. The memory cost, not the invariant question, is what
dominates hosting A + shape 4's analysis.

What hosting A does *not* require: no `BatchSim::new` signature
change, no `PassiveComponent::apply` signature change, no new
struct types. It is the lightest-touch hosting choice mechanically.
The contested question is whether the `Data` invariant tolerates
the fields the mechanism needs — and that question is not
mechanical.

### Hosting B: on a sibling struct owned by `BatchSim`

If per-env state that is not physics should not live on `Data`, the
next obvious place is a per-env sibling struct owned by `BatchSim`,
parallel to `envs: Vec<Data>`. Call it `env_meta: Vec<EnvMeta>` for
the purpose of this chapter; the name is a placeholder and Ch 15 is
welcome to pick a different one. `EnvMeta` holds whatever per-env
state the mechanism needs: a `ChaCha8Rng` for shape 1, a `(traj_id,
step_index)` pair for shape 3, a tape plus read index for shape 4.
`BatchSim::step_all` advances the `EnvMeta` entry for each env at
the appropriate point — incrementing the step index or pulling the
next noise value — and passes it through to `apply`.

The costs of hosting B are concentrated in the passing-through.
`PassiveComponent::apply` currently has the signature

```rust
fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>);
```

Hosting B under any mechanism other than shape 4-with-the-component-
managing-the-tape requires `apply` to see the per-env meta. Two
sub-options exist.

**B1: change the trait signature.** Replace the current signature
with something like

```rust
fn apply(&self, ctx: &ApplyCtx, qfrc_out: &mut DVector<f64>);
```

where `ApplyCtx` carries `&Model`, `&Data`, and `&EnvMeta` (or
equivalents). This is a chassis-wide breaking change. Every
existing `PassiveComponent` impl in the codebase must update its
signature — `DoubleWellPotential`, `OscillatingField`,
`RatchetPotential`, `PairwiseCoupling`, `ExternalField` in the
thermostat crate, the test-fixture impls in `stack.rs` and
`component.rs`, and `LangevinThermostat` itself. Most of these
impls do not care about `EnvMeta`; they currently ignore anything
except `model`, `data`, and the output buffer, and they would
continue to ignore `EnvMeta` under the new signature. The cost is
the signature change itself, not any change to the pure components'
logic. The update is mechanical but wide: every `PassiveComponent`
impl in the workspace, plus every call site in the `cb_passive`
closure machinery.

**B2: thread the meta through `cb_passive`.** The `cb_passive`
closure installed by `PassiveStack::install` is the thing that
actually calls `apply` on each component in the stack. If the
closure holds a mutable reference to the `EnvMeta` for the current
env (captured from the `BatchSim::step_all` call stack), it could
pass a pointer or reference into each `apply` call without the
trait signature changing — for example, via a thread-local, a
`RefCell` on the stack, or a second callback the component can
invoke to read the meta. Each of these is its own kind of ugliness:
thread-locals break under rayon's thread pool in subtle ways; a
`RefCell` on the stack has to survive `Send + Sync`; a second
callback is a trait signature change in disguise, threaded through
a different entry point.

B1 is the more direct version of hosting B; B2's avoidance of the
trait change comes at the cost of mechanisms that are each their
own kind of fragility, and is not obviously better. The cost both
of them carry is that `BatchSim::step_all` has to know which
`EnvMeta` entry to hand to which env's `cb_passive` invocation,
which requires `BatchSim::step_all` to be aware of the meta vector
— a change to `BatchSim`'s internals.

What hosting B preserves: the `Data` invariant, completely. `Data`
stays a physics struct, and `EnvMeta` is where RL-bookkeeping
metadata lives. The mjData-analog framing of `Data` survives
unchanged. A reader who wants to know "what is the physics state of
env $i$" reads `batch.envs[i]`; a reader who wants to know "what
bookkeeping is this env carrying" reads `batch.env_meta[i]`. The
two concepts are in two structs, and the boundary between them is
explicit.

What hosting B imposes: a `PassiveComponent::apply` trait signature
change (under B1) or a harder-to-read threading mechanism (under
B2), plus a `BatchSim` internal change to own and advance the
`env_meta` vector.

### Hosting C: on per-env component instances via `install_per_env`

The third hosting choice uses machinery that already exists in the
thermostat crate. `PassiveStack::install_per_env` at
`sim/L0/thermostat/src/stack.rs:174` (recon-reported) is a method
that takes a count `n` and a factory closure `build_one: FnMut(usize)
-> (Model, Arc<Self>)`, calls the factory `n` times (passing each
call an env index), and installs the resulting stack onto each
model. The return value is an `EnvBatch { models, stacks }`
carrying the `n` independent `(Model, PassiveStack)` pairs. Each
env's stack — and each stochastic component inside that stack — is
a fresh instance, with no aliasing across envs.

The module doc at `stack.rs:20-25` (recon-reported) calls this out
explicitly as the path for parallel-environment construction:
*"Supports parallel-environment construction via
`PassiveStack::install_per_env`, which builds N independent
`(Model, PassiveStack)` pairs from a user-supplied factory.
Decision-3 + N4 enforce that each env's stack is fresh (no aliased
RNG state) via a `debug_assert!` + defensive
`clear_passive_callback` pair."* The debug assertion at
`stack.rs:185-186` (recon-reported) fires in debug builds if the
factory returns a model that already has a `cb_passive` callback
installed — catching misuse loudly in dev. A defensive
`clear_passive_callback` call follows the assertion so release
builds stay correct even when the assertion is compiled out.

What hosting C does, described independently of mechanism: it gives
each environment its own fresh `PassiveStack`, including its own
fresh stochastic-component instance. Any state the component
carries — a stream generator, a counter, a tape read index — is
that env's own, structurally, with no cross-env sharing at the
field level. The chassis already contains the primitive. What it
does not already contain is a `BatchSim::new` path that *calls*
`install_per_env` — the current `BatchSim::new(model: Arc<Model>,
n: usize)` at `batch.rs:73-76` takes a single pre-built model and
clones handles into `n` `Data` instances, never invoking the
factory path.

Hosting C under shape 1: each env's thermostat instance has its own
`Mutex<ChaCha8Rng>`. Because the instance is per-env, the mutex is
structurally uncontended: `cb_passive` is single-threaded within
one env, and there is no other env reaching for the same instance.
The mutex remains a correctness requirement for `Send + Sync` with
the current generator type choice (`ChaCha8Rng` is not `Sync` on
its own) but does not serve any contention purpose. A reader of
`langevin.rs` under hosting C + shape 1 still sees `rng:
Mutex<ChaCha8Rng>` and still has to understand the three questions
named under shape 1 (why interior-mutable, why `Sync`-wrapped, why
uncontended in the happy path); the answers are cleaner than under
the current code (where the mutex also serves a real cross-env
contention purpose) but the ceremony is not zero. Under hosting C,
the per-env isolation might also justify a non-mutex `Sync` wrapper
or a generator type that is `Sync` natively; Ch 15 can revisit
whether C-1's mutex ceremony is a hard cost or a soft one once the
per-env path is committed to.

Hosting C under shape 3: each env's thermostat instance carries a
`traj_id: u64` const field set at construction time by the factory
closure, and a `counter: AtomicU64` that advances with each active
`apply`. The `AtomicU64` is `Sync` natively and lock-free;
`fetch_add` replaces the `lock()` pattern at `langevin.rs:184`
(recon-reported) with a single atomic integer increment. Gating
works as described under shape 3 in Section 1, with the same
early-return-before-counter-advance pattern at `langevin.rs:167-174`
(recon-reported). The noise computation becomes `PRF(master_seed,
traj_id, counter_value, dof_idx)` — a pure function, with master
seed as a const on the component (set by the factory) and all
integers as locals or component fields. No mutable stream state.
No mutex. The `AtomicBool` for the gating flag stays where it is.

Hosting C under shape 4: each env's thermostat instance owns its
own slice of the pre-generated tape. The tape is per-env, the read
index is per-env, both live on the component instance. Same
memory/flexibility costs as shape 4 in general, but with the
per-env isolation property that hosting C gives.

What hosting C preserves: the `Data` invariant (nothing goes on
`Data`), the `PassiveComponent::apply` trait signature (nothing
changes there), and the existing `install_per_env` machinery (no
new primitive needed).

What hosting C requires: a `BatchSim::new` call path that invokes
`install_per_env`. The current constructor does not. The refactor
is: change `BatchSim::new` to accept a factory (or to accept an
already-built `EnvBatch` from a prior `install_per_env` call), so
the `BatchSim` construction path goes through the per-env installer.
This is a `BatchSim` constructor change — not a trait change, not a
chassis invariant question, not a `Data` change — but it is real.
Callers of `BatchSim::new` today pass `(Arc<Model>, n)`; under
hosting C they need to pass a factory instead, or construct the
`EnvBatch` separately and hand it in. Ch 15 or the Part 4 PR plan
is the right place to pick the exact constructor signature; Ch 14
just names the change as a prerequisite for any cell under hosting
C.

### Summary of the hosting axis

Hosting A is the lightest-touch choice mechanically, and its
defensibility turns on how strictly the `Data` invariant is read.
Hosting B preserves the invariant completely at the cost of a
`PassiveComponent::apply` signature change and a parallel
`env_meta` structure owned by `BatchSim`. Hosting C uses machinery
the thermostat crate already contains, preserves both the invariant
and the trait signature, requires a `BatchSim::new` constructor
change, and eliminates cross-env aliasing at the structural level
rather than at the field-value level.

The hosting axis has three live entries for the grid in section 3:
A, B, and C. None is eliminated by the walk.

## Section 3 — The grid

Composing the mechanism and hosting axes produces twelve cells in
principle. Shape 2 collapses (in two directions) into shapes 1 and
3 in Rust, shape 4 is eliminated by memory and flexibility cost,
and the remaining cells fall into a small shortlist. The table below summarizes; each row is a cell labeled
`<hosting>-<mechanism>`. "Chassis changes" counts strictly
structural work, not per-component logic changes.

| Cell | Mechanism | Hosting | Chassis changes | Notes |
|------|-----------|---------|-----------------|-------|
| A-1  | Shape 1 (stateful stream) | On `Data`              | `Data` grows a per-component stream field; `&mut Data` trait change OR nested interior mutability inside `Data` | Couples `Data` to component internals; no natural shape |
| A-3  | Shape 3 (counter + PRF)   | On `Data`              | `Data` grows `step_index: u64` (optionally `traj_id: u64`); PRF implementation (Section 1, three routes) | Trait signature intact; contested by strict reading of the `Data` invariant; live under the loose reading via the `time`-field precedent |
| A-4  | Shape 4 (pre-generated)   | On `Data`              | `Data` grows per-env tape plus read index; memory cost                     | Trait signature intact; memory scales with run length |
| B-1  | Shape 1                   | On sibling `EnvMeta`   | `PassiveComponent::apply` signature change; new `EnvMeta` struct            | Stream still wrapped for `Send + Sync`; meta lives cleanly outside physics |
| B-3  | Shape 3                   | On sibling `EnvMeta`   | Signature change; new struct; PRF implementation (Section 1, three routes)  | Architecturally clean — separates physics (`Data`) from RL metadata (`EnvMeta`) — at the cost of the widest trait-level refactor on the grid |
| B-4  | Shape 4                   | On sibling `EnvMeta`   | Signature change; new struct; memory cost                                   | Same memory problem as A-4 with an added signature-change cost |
| C-1  | Shape 1                   | Per-env component instance (`install_per_env`) | `BatchSim::new` rewired to call `install_per_env`                           | Current code's shape with the stream properly per-env; mutex stays as ceremony |
| C-3  | Shape 3                   | Per-env component instance                    | `BatchSim::new` rewired; PRF implementation (Section 1, three routes)        | Lock-free `AtomicU64` replaces `Mutex<ChaCha8Rng>` under Route 2; no `Data` change; no trait change |
| C-4  | Shape 4                   | Per-env component instance                    | `BatchSim::new` rewired; memory cost                                         | Tape lives on the component; per-env isolation structural |

Collapses and eliminations:

- **Cell A-1 does not have a natural shape.** Hosting a stream on
  `Data` requires either a trait signature change to `&mut Data`
  (which wins none of A's usual advantages) or interior mutability
  on `Data` reintroducing the mutex-as-ceremony cost shape 1 was
  supposed to avoid. It stays in the table for completeness but is
  not in the Ch 15 shortlist.
- **Shape 4 cells (A-4, B-4, C-4) are eliminated by memory and
  flexibility cost.** The 16M-step rematch budget makes the tape
  size noticeable and the inflexibility to varying episode lengths
  forces additional counter state that gives shape 4 no advantage
  over shape 3. Shape 4 is listed for honest coverage of Ch 10's
  enumeration and is not in the Ch 15 shortlist.
- **Cells B-1 and C-1 are the same mechanism with different
  hosting.** Both preserve the current stateful-stream mechanism.
  B-1 imposes a chassis-wide `PassiveComponent::apply` signature
  change; C-1 imposes only a `BatchSim::new` constructor change.
  C-1 dominates B-1 on the shape-1 hosting question for the
  current single-stochastic-component world; if a second stochastic
  component is added later, B-1's already-paid trait signature
  change accommodates the second component without re-opening the
  `BatchSim::new` rewire question, and the comparison reopens. For
  the Ch 15 shortlist, B-1 is out; the "reopens later" observation
  is noted as a Part 4 concern.
- **Cells B-3 and C-3 are the same mechanism with different
  hosting.** Both commit to the counter-based PRF mechanism. B-3
  puts the `(traj_id, step_index)` pair on an `EnvMeta` sibling
  struct and changes the `PassiveComponent::apply` trait signature
  to surface it. C-3 puts the counter on the per-env component
  instance as an `AtomicU64` and requires only a `BatchSim::new`
  constructor change. Both are architecturally clean in different
  ways — B-3 via struct separation of physics from bookkeeping, C-3
  via "state lives with the thing that uses it." Ch 15's pick
  between B-3 and C-3 is one of the chapter's main design calls.
- **Cell A-3** is the `Data`-hosted counter-based option. It is
  live iff the loose reading of the `Data` invariant wins over the
  strict reading. Ch 15 is the place to adjudicate that reading.

The Ch 15 shortlist therefore contains four cells: A-3, B-3, C-1,
and C-3. One under shape 1 (C-1), three under shape 3 (A-3, B-3,
C-3). Ch 15 has to pick within and across both shapes — which
mechanism wins, and for whichever shape wins, which hosting.

Ch 14 does not pick. Ch 15 does.

## What this chapter does not decide

It does not pick between shape 1 and shape 3 — the central
mechanism question. It does not pick between hosting A, B, and C
for whichever shape wins. It does not name a PRF crate. It does
not specify the `BatchSim::new` constructor signature. It does not
write the `EnvMeta` struct definition for the hosting-B path. It
does not decide how tightly the `Data` invariant should be read.
It does not specify the regression-test shape that Ch 13 named as
a Ch 15 requirement. All of those are Ch 15's questions, to be
made against the grid this chapter has laid out.

What this chapter does decide, if "decide" is the right word, is
the shape of the space Ch 15 has to navigate. The axes are
independent, the constraints from Part 1 and Part 2 are
load-bearing for every cell, shape 2 collapses (in two directions)
into shapes 1 and 3 in Rust, shape 4 is eliminated by memory and
flexibility cost, and the Ch 15 shortlist has four cells. The
thinking pass against this chapter is where the grid should be
pressure-tested — whether a fifth mechanism exists, whether the
invariant reading matters less than this chapter makes it, whether
the `install_per_env` constructor rewire is actually shared between
C-1 and C-3 or interacts differently under each — before Ch 15
makes the call.
