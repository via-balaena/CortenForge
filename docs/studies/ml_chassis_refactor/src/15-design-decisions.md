# Design decisions

Chapter 14 laid out the 2-axis design grid and handed Ch 15 a
four-cell shortlist: A-3, B-3, C-1, C-3. This chapter picks. It
also specifies the four design calls that follow from the pick:
the PRF implementation route, the per-env seed derivation question,
the gating-trait preservation mechanism, and the regression test
that Ch 13 named as a Ch 15 requirement.

The pick is **C-3**: shape 3 (counter-based PRF) hosted on per-env
component instances via `install_per_env`. The argument takes four
steps. First, shape 3 wins the mechanism axis over shape 1 —
conditional on picking PRF implementation Route 2 (the manual
ChaCha8 block function), which the chapter argues for in Section
2. Second, hosting A is eliminated on the strict reading of the
`Data` invariant. Third, hosting C wins over hosting B on balance
once the gating-under-FD invariant from the current code is
accounted for — this is the step that surfaced during Ch 15's
recon and forced a reversal from an earlier direction. Fourth,
the shortlist collapses to C-3.

The four smaller design calls fall out of C-3 more or less
automatically. Per-env seed derivation disappears as a question
entirely under shape 3 (there are no per-env derived seeds;
`traj_id` is just the env index and the master seed is shared).
Gating preservation is unchanged from the current code's pattern
(the `AtomicBool` flag on the component, early-return before the
counter advance). The regression test is a parallel-matches-
sequential assertion using the Ch 13 oracle, which under shape 3
passes by construction because the PRF is a pure function with no
order-dependence. The PRF implementation is the one design call
that requires a substantive choice — Route 2 out of the three
routes Ch 14 named.

The chapter ends with an explicit scope-discipline paragraph
naming what it does not decide — most notably, it does not write
the actual PRF code, specify the exact `BatchSim::new` signature,
or address how a second stochastic component (when one is added
later in the thermo-computing line) should reuse the pattern. All
of those are Part 4 PR concerns.

## What Ch 15 inherits from Ch 14

Ch 14's grid in Section 3 produces a four-cell shortlist after
walking the 2×4 mechanism × hosting grid and applying its
collapses and eliminations:

- **A-3** — shape 3 (counter + PRF) hosted on `Data` directly. Live
  iff the loose reading of the `Data` invariant at `data.rs:27-30`
  (recon-reported) is accepted over the strict reading. Ch 14
  explicitly deferred the reading to Ch 15.
- **B-3** — shape 3 hosted on a sibling `EnvMeta` struct owned by
  `BatchSim`, with a `PassiveComponent::apply` trait signature
  change to surface the `EnvMeta` via an `ApplyCtx` parameter.
- **C-1** — shape 1 (per-trajectory stateful RNG, the current
  code's mechanism) hosted on per-env component instances via the
  existing `PassiveStack::install_per_env` machinery at
  `sim/L0/thermostat/src/stack.rs:174` (recon-reported). Requires
  a `BatchSim::new` constructor rewire to invoke `install_per_env`.
- **C-3** — shape 3 hosted on per-env component instances. Counter
  lives on the component as an `AtomicU64`. Requires the same
  `BatchSim::new` constructor rewire as C-1, plus a PRF
  implementation (Ch 14 Section 1 names three routes; Section 2
  of this chapter picks Route 2).

Ch 14 explicitly noted that the `install_per_env` constructor
rewire is shared between C-1 and C-3 and should not be
double-counted when comparing them. It also noted that the main
race is between C-1 (current code's shape, minimum diff) and the
shape-3 family (architectural purity, implementation cost).

The four design calls Ch 15 has to make, per the Phase 3 roadmap:

1. **The cell pick.** Which of the four shortlist cells wins.
2. **Per-env seed derivation primitive.** How each env gets its
   own RNG-level independence. Ch 14 noted this question matters
   under shape 1 (stateful per-trajectory RNG) but flagged that
   shape 3 may not need it. Ch 15 resolves that.
3. **Gating-trait preservation mechanism.** How the `Stochastic`
   sibling trait at `sim/L0/thermostat/src/component.rs:101-108`
   (recon-reported) and its FD invariant survive the refactor. Ch
   14 asserted that all four mechanisms in Ch 10's enumeration
   satisfy the gating constraint; Ch 15 specifies the exact
   mechanism under the chosen cell.
4. **Regression test spec.** Chapter 13 named a test as a Ch 15
   requirement: "installs a stochastic component, runs N parallel
   envs, and asserts reproducibility across runs with the same
   master seed. The sequential fallback is the oracle." Ch 15
   specifies what this test does, what it asserts, and what the
   pass criterion is.

There is a fifth decision Ch 15 is making that does not appear on
the Phase 3 roadmap's list but is load-bearing for the cell pick:
**which PRF implementation route.** Ch 14's Section 1 (post the
Round 3 patch) named three routes and deferred the pick to Ch 15.
Route selection is not truly separable from the cell pick — the
argument for C-3 over C-1 depends on Route 2 being available and
picked, and Route 1 would make C-3 strictly worse than C-1 on
reader clarity. Section 2 argues for Route 2 after Section 1 has
picked the cell.

## Section 1 — The cell pick: C-3

The pick is C-3. The argument has four steps: shape 3 wins the
mechanism axis (conditional on Route 2), hosting A is eliminated
on the strict reading of the `Data` invariant, hosting C wins
over hosting B on balance once the gating-under-FD invariant is
accounted for, and the shortlist collapses to C-3.

### 1.1 Shape 3 wins the mechanism axis

The shape 1 vs shape 3 question is the central mechanism call and
the chapter addresses it first because the hosting analysis is
different under each shape. Under shape 1 the C-1 candidate is the
only survivor; under shape 3 the choice is between A-3, B-3, and
C-3. Picking the shape first tells the reader which hosting
sub-argument matters.

Ch 14 Section 1 framed shape 1 as "the mechanism the current code
already uses, minus the shared-state defect." This is accurate.
Shape 1 is a per-trajectory stateful RNG — each env has its own
stream, seeded from a disjoint initial seed, advancing with each
draw. Gating is the existing flag-check early-return pattern at
`sim/L0/thermostat/src/langevin.rs:167-174` (recon-reported). The
refactor cost under shape 1 is small: the current code already has
the right structure, and C-1 just needs `BatchSim::new` rewired to
call `install_per_env` so each env gets its own thermostat
instance.

Shape 3 is a counter-based PRF. The component carries
`(master_seed, traj_id, counter, active_flag)` and computes noise
at each step as a pure function of those integers plus the DOF
index. There is no "stream state" that advances per draw in the
sense shape 1 has; instead, an `AtomicU64` counter advances per
apply call and the PRF produces uniform output at the current
`(traj_id, step_index, dof)` coordinate. Under PRF implementation
Route 2 (the manual ChaCha8 block function argued for in Section
2), no `ChaCha8Rng` instance is stored on the component at all —
just the 32-byte master key (the ChaCha8 key expanded once from
the user-supplied `u64` master seed at construction time), and
the component's fields are `(gamma, k_b_t, master_key, traj_id,
counter, active)`, all of which are either `Copy` or `Sync`
natively.

The question is whether shape 3's purity is worth the PRF
implementation cost (Section 2 argues for Route 2, which is ~50 to
100 lines of well-documented ChaCha8 block cipher code in the
thermostat crate). Four observations decide it.

**First, the reader-clarity argument.** Ch 14 named three things a
reader has to understand about shape 1's RNG field: (a) why is
this interior-mutable, (b) why is this `Sync`-wrapped, (c) why is
this uncontended in the happy path. Under C-1 with `install_per_env`
giving each env its own thermostat, the answers are cleaner than
under the current code (the third question reduces to "because
`cb_passive` is single-threaded within one env") but the three
questions remain. Under C-3 Route 2, the RNG field does not exist
on the component; the component fields are integers and a master
key, all of which satisfy `Send + Sync` without an interior-
mutability wrapper. The PRF function itself is a self-contained
pure function the reader can study in isolation. The cost of
understanding it is paid once rather than every time a reader
opens a stochastic component file.

Under the "beginners can see the architecture clearly" preference
from the project's working principles, localized complexity in a
tested pure function wins on balance against distributed complexity
in every stochastic component's RNG field. The cost is ~50-100
lines of well-documented block cipher code owned by the thermostat
crate and a verification test against `rand_chacha`; the benefit
is that a reader opening any stochastic component file sees
integer fields and a PRF call, not a mutex. Shape 3 + Route 2
takes the better trade on this dimension.

**Second, the foundational-over-patch argument.** The project's
working principles favor "foundational fixes over quick patches."
C-1 is the quick-patch answer: it keeps the existing mutex pattern
and wires it through `install_per_env`. Under C-1, future
stochastic components (the thermo-computing line's planned thermal
ratchets, non-equilibrium baths, and Gibbs-sampler-adjacent
components) each carry the same 2-line mutex idiom
(`rng: Mutex<ChaCha8Rng>` field, `self.rng.lock()` call in
`apply`) that the current Langevin code carries. Under C-3 + Route
2, future stochastic components each call `chacha8_block` by
convention. Neither approach is chassis-enforced — C-3's
convention is documented rather than trait-encoded (Section 7
declines to introduce a `CounterBasedStochastic` trait until a
second component forces the question). The difference is not
"shape 1 requires the mutex dance and shape 3 does not"; both
shapes require per-component code that follows a pattern. The
difference is that shape 3's per-component code reads as
arithmetic (constructor args + counter + PRF call) while shape
1's reads as ceremony (mutex + lock + careful drop semantics).
For the "beginners see the architecture clearly" criterion, the
shape-3 reading is preferred.

**Third, the gating-trait preservation argument.** Under shape 1,
gating preservation requires the early-return pattern to happen
*before* the mutex lock (so an inactive apply does not
unnecessarily acquire a lock, and so the RNG stream state does not
advance). The current code at `langevin.rs:146` and `langevin.rs:167-174`
(recon-reported) implements this correctly. Under shape 3, gating
preservation requires the early-return to happen before the
counter's `fetch_add`. This is the same pattern, slightly simpler,
because there is no mutex to avoid — the counter is an `AtomicU64`
and the early-return is an ordinary conditional. The gating
invariant Section 4 specifies is preserved by construction under
both shapes, but shape 3's implementation is a single conditional
protecting a single atomic operation, which is cleaner than shape
1's implementation protecting a mutex lock. Small win, but real.

**Fourth, the R34 chassis-overbuild philosophy.** The working
principles describe the chassis philosophy as "overbuild the
chassis, bolt-on aftermarket." Shape 1's mutex pattern is a
chassis-level feature of every stochastic component — every future
aftermarket stochastic component inherits the pattern. Shape 3's
pure PRF primitive is a chassis-level feature of the thermostat
crate — every future stochastic component inherits a pure function
to call with its own `(master_seed, traj_id)` coordinates. The
second is "overbuilt chassis" in the honest sense: the chassis
has a primitive that aftermarket components use. The first is
"aftermarket components each reinvent the same pattern." The
chassis philosophy prefers the second reading.

**The conditional on Route 2 matters here.** Under Route 1 (using
`rand_chacha`'s `set_word_pos` API), shape 3 keeps a
`Mutex<ChaCha8Rng>` on the component because the seeking API
requires `&mut self` and `ChaCha8Rng` is not `Sync`. A reader
under Route 1 sees `Mutex<ChaCha8Rng>` *plus* the seek-based
counter semantics — more complexity than shape 1, not less. The
reader-clarity argument fails under Route 1. This is the sense in
which the cell pick and the PRF route pick are not separable:
C-3 only wins over C-1 if Route 2 is on the table. Section 2
commits to Route 2 as a load-bearing consequence of this Section 1
argument.

Under the four observations above — reader clarity, foundational
over patch, gating simplicity, R34 chassis overbuild — **shape 3
wins the mechanism axis**, conditional on Route 2 being the PRF
implementation choice.

### 1.2 Hosting A is eliminated on the strict reading of the `Data` invariant

Ch 14 Section 2 framed the `Data` invariant question as "genuinely
contested": a strict reading disqualifies shape 3 on `Data`
(because `step_index` is RL-discretization bookkeeping, not
physics), and a loose reading allows it (because `Data` already
has `time: f64` at `data.rs:520` as a clock field, and `step_index`
is arguably a discrete sibling of `time`). Ch 14 deferred the
reading to Ch 15. This section adjudicates.

**Ch 15 picks the strict reading.** The argument has three parts.

First, the `time` / `step_index` equivalence Ch 14 flagged as
"suggestive rather than dispositive" does not reach the threshold
needed to justify growing a chassis struct. `time` earns its
place on `Data` by its *consumer reach*: it is the independent
variable of the Langevin SDE, and components like
`OscillatingField` and `RatchetPotential` read it as a physics
quantity from *outside* the thermostat-internal noise-generation
path. `Data` is the chassis struct that non-thermostat consumers
touch, and `time` needs to live there because physics-computing
code across multiple components reads it. `step_index` does not
have this property. It would be read only by the PRF path inside
stochastic components — a thermostat-internal concern with no
reach outside the stochastic-component subset of the codebase.
Hosting a thermostat-internal field on `Data` means hosting an
internal implementation detail on a chassis struct that many
non-thermostat consumers touch, which is the opposite of the
`PassiveComponent`-as-aftermarket framing. The precedent `time`
sets is that `Data` tolerates fields read by physics across the
codebase, not fields read only by one subset of components.

Second, the precedent-extension argument has a slippery slope
problem. Once `step_index` is on `Data` because "`Data` already has
`time`, and `step_index` is the discrete sibling," a future
request to put `traj_id` on `Data` follows by the same logic (it's
constant, it's a `u64`, it's a per-env identifier). Once `traj_id`
is on `Data`, a future request to put `master_seed` on `Data`
follows similarly. Each individual extension is small; the
cumulative effect is that `Data` grows a second category of
fields (RL-bookkeeping metadata) alongside its original category
(physics state). The `Data` invariant at `data.rs:27-30`
(recon-reported) — "qpos and qvel are the ONLY state variables.
Everything else (xpos, xquat, qfrc_*, etc.) is COMPUTED from them
via forward dynamics" — exists precisely to prevent this kind of
boundary creep. Accepting `step_index` on `Data` is the first step
down a slope the invariant was designed to stop.

Third, a clean alternative exists. Hosting C does not touch `Data`
at all. `step_index` (as an `AtomicU64` counter) and `traj_id` (as
a `const u64`) live on the component instance, where they are
used. The `Data` invariant is preserved. The `BatchSim::new`
rewire that hosting C requires is a real cost, but it is a cost
in a different part of the chassis — a constructor signature, not
a chassis-level state struct — and the constructor change does
not violate any documented invariant. When a clean alternative
exists, there is no reason to violate the boundary; the R34
philosophy's "respect the boundaries so future growth has clear
lines" principle applies directly.

**Under the strict reading, cell A-3 is eliminated.** The loose
reading is not wrong — it is defensive: it says "we *can* do this
without technically breaking anything documented." But the strict
reading is normative: it says "we *should not* do this because the
chassis contract is load-bearing for future growth and a clean
alternative exists." Under "A-grade or it doesn't ship" and
"foundational fixes over quick patches," the normative reading
wins.

### 1.3 Hosting C wins over hosting B on balance

This is the step that forced a reversal during Ch 15's recon and
deserves an explicit walk. The argument has two independent
stages: first, B-3's natural form breaks the gating-under-FD
property, and second, repairing B-3 to fix FD collapses its
main benefit to triviality. Both stages are needed to dismiss
B-3 honestly.

The earlier reading of the B-3 vs C-3 question was that B-3 was
preferable on R34-chassis-overbuild grounds: the trait signature
change to `apply(&self, ctx: &ApplyCtx, qfrc_out)` would make
per-env metadata a chassis-level concern that every future
stochastic component inherits for free, rather than a per-
component convention. The reading was plausible on paper; it did
not survive contact with the gating-under-FD property the current
code already has.

**The gating-under-FD property.** The test
`apply_does_not_advance_rng_when_stochastic_inactive` at
`sim/L0/thermostat/src/langevin.rs:311-363` (recon-reported)
operationalizes an invariant: when a stochastic component is
disabled via the `Stochastic::set_stochastic_active(false)` path,
its `apply` method must produce its deterministic contribution
without advancing any RNG state. Post-re-enable, the noise stream
is exactly where it was at disable time. This property is
load-bearing for the Phase 5 finite-difference path: two FD
evaluations under `PassiveStack::disable_stochastic` (the RAII
guard at `stack.rs:231`, recon-reported) must produce identical
deterministic forces, which means no RNG state can advance between
them. The current code preserves this by checking the flag and
early-returning before the `self.rng.lock()` call at
`langevin.rs:184` (recon-reported).

**Stage 1: B-3 with the step counter hosted on `EnvMeta` breaks
gating-under-FD.**

The natural reading of B-3 puts `(traj_id, step_index)` on a
sibling `EnvMeta` struct owned by `BatchSim`. It is the natural
reading because `EnvMeta`'s purpose — hold per-env state that
sits outside `Data` and outside the component instance — is
exactly what a per-env step counter is: per-env state that is
not physics and not a component-internal detail. Putting the
counter anywhere *else* under B-3 is possible (see stage 2) but
defeats the framing that motivates the `EnvMeta` struct in the
first place.

Under this reading, `BatchSim::step_all` advances
`EnvMeta.step_index` at each physics step and passes the
`EnvMeta` reference to `apply` via the new `ApplyCtx` parameter.
The thermostat reads `ctx.env_meta.step_index` as its current PRF
coordinate and produces noise.

Now consider the FD use case. `PassiveStack::disable_stochastic`
returns a `StochasticGuard`. The user wraps an FD block in the
guard, runs `n_fd_steps` forward integrations (baseline rollout
and perturbed rollout, two rollouts total), and drops the guard
to re-enable stochastic. Under B-3 with the counter on `EnvMeta`,
`BatchSim::step_all` advances `EnvMeta.step_index` at every
physics step regardless of whether the thermostat is gated —
because `BatchSim::step_all` is the thing that owns `EnvMeta`
and has no visibility into the stack's stochastic state. After
the FD block, `EnvMeta.step_index` has advanced by `2 * n_fd_steps`
(both rollouts). Post-FD, stochastic is re-enabled and the next
apply reads PRF at the shifted step_index.

**This breaks the gating-under-FD property.** Under the current
code, post-FD the RNG stream state is exactly where it was
pre-FD. Under B-3 with counter on `EnvMeta`, post-FD the PRF
coordinate is shifted by `2 * n_fd_steps` from where it would have
been without FD. The noise sequence a user sees after re-enable
is different from the noise sequence they would have seen without
the FD block. This is a property regression relative to the
current code — not a subtle one, and not one that can be
advertised as "shape 3 has different FD semantics."

**The fixes are all unattractive.**

- Option (i): `BatchSim::step_all` checks the stack's stochastic
  state before advancing `EnvMeta.step_index`, and skips the
  advance when the stack is disabled. This couples `BatchSim`
  (in sim-core) to the stack's stochastic state (in thermostat),
  which is a dependency-direction inversion: sim-core would need
  to know about thermostat-crate concepts. The dependency graph
  today has thermostat depending on sim-core, not the reverse.
  Breaking this direction is a wider chassis change than C-3's
  `BatchSim::new` rewire.
- Option (ii): `StochasticGuard` (in the thermostat crate)
  snapshots `EnvMeta.step_index` at disable time and restores it
  at drop. This requires `StochasticGuard` to know about
  `EnvMeta`, which lives on `BatchSim`, which is in sim-core.
  Same dependency-direction problem as option (i), just in the
  other direction.
- Option (iii): Keep the counter on the component rather than on
  `EnvMeta`. The component owns both the gating flag and the
  counter, so the early-return pattern at `langevin.rs:167-174`
  protects both together. No coupling across crates. No
  dependency-direction issue. But under this option, the counter
  is on the component — which is exactly what C-3 does, and B-3's
  "EnvMeta owns per-env state" benefit disappears.

**Stage 2: repairing B-3 via option (iii) collapses its main
benefit.** Option (iii) is the only fix for the FD regression
that does not force a cross-crate coupling. Under it, B-3 puts
only `traj_id: u64` and `master_seed: u64` on `EnvMeta` (both
constants, no mutation — nothing that FD has to preserve). The
component holds `counter: AtomicU64` and `active: AtomicBool`
(both mutable, both in the early-return path, both preserving
FD by the same mechanism the current code already uses). The
B-3 side of the comparison now reduces to: "EnvMeta carries two
`u64` constants that the factory populates once per env, and the
trait signature change lets components read them via
`ctx.env_meta`." The C-3 side reduces to: "the factory passes the
two `u64` constants as constructor arguments on each env's
component instance."

At this point the FD-invariant argument has done its work: B-3
is no longer *broken*, but only because it has been refactored
into a shape where its distinguishing feature (per-env state
hosted on a dedicated struct) is carrying two constants. A
second, independent argument now has to show why that shape is
not worth the trait signature change.

**The remaining B-3 benefit is very small.** A future stochastic
component under B-3 reads `ctx.env_meta.traj_id` and
`ctx.env_meta.master_seed`. Under C-3, the same future component
takes `master_seed` and `traj_id` as constructor arguments from
the factory. Both are two pieces of information the component
needs; the difference is where they are accessed (at apply-time
from a struct reference vs at construction-time as arguments).
The construction-time-argument approach is simpler on balance: no
new type (`ApplyCtx`), no trait signature change, no 12-impl
mechanical refactor, and the existing pattern `LangevinThermostat::new(gamma, k_b_t, seed)`
at `langevin.rs:102` (recon-reported) already takes constructor
arguments for its chassis-level configuration. Adding two more
arguments (`master_seed`, `traj_id`) is mechanical and requires
no new chassis concept.

**Under hosting C, B-3's chassis-overbuild benefit becomes
construction-time argument passing, which is already how the
chassis works.** The trait signature change buys nothing beyond
what the existing constructor pattern already provides. The cost
— 12 `PassiveComponent` impls updated (6 production at
`double_well.rs:142`, `oscillating_field.rs:139`, `ratchet.rs:142`,
`pairwise_coupling.rs:167`, `external_field.rs:61`, and
`langevin.rs:133`, plus 6 test impls in `stack.rs:307,316,327` and
`component.rs:124,137` and `tests/langevin_thermostat.rs:260`) —
is real and unrecovered because the benefit does not materialize.

**Hosting C wins over hosting B on balance.** Not because C is
simpler mechanically (it is, but that alone would be a C-1-style
minimum-diff argument), but because the two stages above show
B-3's stated advantage does not survive implementation: stage 1
shows the natural reading breaks gating-under-FD, and stage 2
shows the only repair leaves B-3 with a trait signature change
that buys nothing beyond what constructor arguments already
provide. The conclusion rests on both stages, not on either
alone.

### 1.4 The shortlist collapses to C-3

Putting the three sub-arguments together:

- Shape 3 wins the mechanism axis over shape 1 (Section 1.1),
  under Route 2 for the PRF implementation. C-1 is eliminated.
- Hosting A is eliminated on the strict reading of the `Data`
  invariant (Section 1.2). A-3 is out.
- Hosting C wins over hosting B on balance once the
  gating-under-FD invariant is accounted for (Section 1.3). B-3
  is out.

**The only remaining cell is C-3.** Shape 3 + per-env component
instance hosting via `install_per_env` + Route 2 for the PRF
implementation. The `BatchSim::new` constructor gets rewired to
accept a factory closure (or equivalent) so the construction path
goes through `PassiveStack::install_per_env` at
`sim/L0/thermostat/src/stack.rs:174`. Each env's thermostat
instance holds `(gamma, k_b_t, master_key, traj_id, counter,
active)` as fields. The PRF primitive is a pure function in a
new module inside the thermostat crate.

**What would have changed the pick.** The pick rests on four
specific calls Ch 15 made, and it is worth naming the
counterfactuals honestly. The pick would have gone to C-1 if
Route 2 had been off the table (if no honest ChaCha8 PRF
implementation were available, Section 2's comparison would have
resolved to keeping the mutex pattern rather than adopting Route
1's worse reader-clarity profile). It would have gone to A-3
under the loose reading of the `Data` invariant — a defensible
reading this chapter declined on R34 chassis-overbuild grounds
but which another reader could accept. It would have gone to B-3
in a world where the FD gating invariant was not load-bearing
(if the thermostat crate did not depend on FD-stable RNG state
for the Phase 5 finite-difference path, stage 1 of Section 1.3's
argument would not apply, and the stage 2 constructor-arg-vs-
ApplyCtx comparison would be close enough to make B-3's
chassis-overbuild story competitive). C-3 is the pick given the
four specific calls Ch 15 made, not a forced conclusion.

## Section 2 — PRF implementation: Route 2 (manual ChaCha8 block function)

The pick is **Route 2**: implement the ChaCha8 block function
directly in the thermostat crate as a stateless pure function.
Ch 14 Section 1 (post the Round 3 patch) named three routes:

- **Route 1:** use `rand_chacha 0.9.0`'s existing
  `ChaCha8Rng::set_word_pos(u128)` seek API (verified at
  `rand_chacha-0.9.0/src/chacha.rs:213` during Ch 15 recon) to
  get counter-based PRF semantics on top of the chassis PRNG,
  with no new dependency.
- **Route 2:** implement the ChaCha8 block function directly in
  the thermostat crate as a ~50-100 line stateless pure function,
  verifiable against `rand_chacha`'s output for correctness.
- **Route 3:** depend on a counter-based PRF crate such as
  `aprender-rand` (Philox 4x32-10 at version 0.29.0) or `squares`
  (smaller counter-RNG at 0.1.1).

### 2.1 Route 1 undoes C-3's architectural win

The Section 1.1 argument for shape 3 over shape 1 rests on reader
clarity: C-3 eliminates the mutex-ceremony complexity of shape 1's
RNG field. Route 1 puts the ceremony back. `ChaCha8Rng::set_word_pos`
takes `&mut self`, and `ChaCha8Rng` is not `Sync`, so a component
using Route 1 must hold its RNG in a mutex or equivalent interior-
mutability wrapper. A reader under Route 1 sees
`Mutex<ChaCha8Rng>` on the component *and* has to learn the
seek-based counter semantics. That is more complexity than shape
1, not less. The argument for C-3 over C-1 fails under Route 1.

Route 1 is listed for honesty — it is a real route that uses an
existing API in an existing workspace crate — but adopting it
would undo the Section 1.1 pick. **Route 1 is eliminated on the
grounds that it defeats the purpose of picking shape 3.**

### 2.2 Route 3 adds external maintenance burden for no concrete benefit

Route 3 takes a dependency on a counter-based PRF crate.
`aprender-rand` implements Philox 4x32-10 at version 0.29.0 from
the `paiml/trueno` repository. `squares` is a smaller crate at
0.1.1 advertising itself as "faster than Philox." Both exist,
both are usable in principle, and both are real Rust crates
published on crates.io.

The cost of Route 3 is that the thermostat crate takes on an
external dependency for a load-bearing chassis primitive, and
the maintenance trajectory of that dependency is outside the
project's control. `aprender-rand` is at 0.29.0 — a high minor
version suggests active development, but the crate lives in an
ecosystem (`paiml/trueno`) that is not in the `rand` family and
does not have the cross-ecosystem visibility that `rand_chacha`
has. `squares` at 0.1.1 is small and unclear in adoption.
Neither crate is in the "take a dependency and forget about it"
category that `rand_chacha` is in.

The benefit Route 3 would buy is that the thermostat crate does
not own the PRF primitive. But Route 2's ~50-100 lines of
ChaCha8 block cipher code is a well-documented, single-file,
testable primitive — owning it is cheap. Owning it in exchange
for not taking an external dependency on a less-established
crate is a good trade for a research codebase that values
dependency minimalism.

**Route 3 is eliminated on the grounds that the benefit (not
owning the primitive) is small compared to the cost (taking a
dependency on a less-established crate), and Route 2 provides an
alternative where the cost is localized.**

### 2.3 Route 2 is the honest answer

Route 2 implements the ChaCha8 block function directly as a
stateless pure function in a new module of the thermostat crate.
The module file name is a Part 4 PR concern (`prf.rs` or
`chacha8_prf.rs` are both defensible). What Ch 15 specifies is
the shape of the module, the function signature, the correctness
verification approach, and the Gaussian transform the Langevin
thermostat applies on top of the PRF output. The exact bit
layouts and type choices below are provisional — Part 4 refines
them against the final `rand_chacha` version and the actual
thermostat crate import surface — but the load-bearing
characteristics (pure function, 256-bit key, 64-byte output per
call, verifiable against `rand_chacha`) are fixed by this
section.

**Shape of the module.** A single file holding the ChaCha8 block
function as a pure function, plus a helper that encodes a
`(traj_id, step_index)` coordinate into the block counter, plus
tests. No types that hold state. No `impl` blocks on structs.
The module is a collection of free functions, and any callers
(the Langevin thermostat initially, other stochastic components
later) use them directly.

**ChaCha8 background.** ChaCha8 is a stream cipher with a
256-bit (32-byte) key, a 64-bit nonce, and a block counter. Each
block produces 64 bytes (16 `u32`s, or equivalently 8 `u64`s) of
uniform pseudorandom output. `rand_chacha 0.9.0` exposes this as
`ChaCha8Rng` with a combined 68-bit word position covering block
counter × word-in-block (see `rand_chacha-0.9.0/src/chacha.rs:191`,
`get_word_pos` → `u128`); Route 2's PRF uses the same underlying
cipher with a simpler API surface — one call produces one block.

**Function signature.** The core PRF primitive takes a 256-bit
key and a block counter, and produces 64 bytes of output:

```rust
/// Compute one ChaCha8 block keyed by `key` at block counter
/// `block_counter`. The output is 64 bytes (one ChaCha block)
/// of uniform pseudorandom data. This is a pure function: the
/// same inputs always produce the same output.
pub fn chacha8_block(key: &[u8; 32], block_counter: u64) -> [u8; 64];
```

The key is 32 bytes because that is what ChaCha8 takes; passing
a `u64` would require an in-function expansion step and would
obscure which expansion rule the function uses. A helper turns
the thermostat's `u64` master seed into a 32-byte key:

```rust
/// Expand a `u64` master seed into a 32-byte ChaCha8 key using
/// the same rule `rand_chacha`'s `SeedableRng::seed_from_u64`
/// uses internally. This is the key expansion Route 2 owns; the
/// verification test in this module asserts it matches the
/// expansion `ChaCha8Rng::seed_from_u64(master_seed)` produces.
pub fn expand_master_seed(master_seed: u64) -> [u8; 32];
```

And a helper encodes the `(traj_id, step_index)` coordinate into
the block counter. **Note: one block is enough for up to 8 DOFs**
(64 bytes = 8 `u64`s, and Box-Muller turns one pair into two
Gaussians, so one block yields 8 Gaussians). The block counter
therefore encodes the `(traj_id, step_index)` pair, not the DOF
index:

```rust
/// Encode a (traj_id, step_index) pair into a block counter.
/// The encoding reserves high bits for traj_id and low bits for
/// step_index, with enough gap between successive traj_id
/// values that two envs never read overlapping blocks at the
/// rematch configuration (32 envs × 500K steps).
pub fn encode_block_counter(traj_id: u64, step_index: u64) -> u64;
```

The exact bit layout is a Part 4 PR concern. The constraint is
that each env gets a large contiguous region of the block
counter space and the regions do not overlap. At 32 envs and
500K steps per env, reserving 24 bits for `step_index` (covering
up to 16M steps per env, with margin) and 40 bits for `traj_id`
(covering up to 10^12 envs) leaves ample room.

**Thermostats with more than 8 DOFs** need multiple blocks per
apply. The wrapper in Section 4 handles this by iterating the
block counter: the first block covers DOFs 0-7, the second
covers 8-15, and so on. `encode_block_counter_for_dof_group`
would take `(traj_id, step_index, dof_group)` where
`dof_group = dof_idx / 8`. For the 6-DOF SR particle of the D2c
rematch, one block per apply is sufficient and the second-block
path is dead code. Part 4 decides whether to ship the general
case or the optimized single-block case.

**Correctness verification.** The Route 2 implementation is
verifiable against `rand_chacha`'s output. For a given master
seed and block counter, `chacha8_block(expand_master_seed(seed), counter)`
produces 64 bytes. The test cross-checks these bytes against
seeding `ChaCha8Rng::seed_from_u64(seed)` and using
`set_word_pos(counter * 16)` to seek to the same block, then
reading 64 bytes. The equality check on `[u8; 64]` runs on
every `cargo test` of the thermostat crate and catches any drift
between Route 2's implementation and the reference. The test
covers (seed, counter) combinations including edge cases (seed
= 0, counter = 0, counter at block boundary, counter near
`u64::MAX`) and any additional combinations Part 4 adds during
implementation.

This verification is the key safety net for owning the primitive.
Ch 15 does not ask the reader to trust that ~50-100 lines of
hand-written ChaCha8 code is correct on its own — the correctness
claim is reduced to "matches `rand_chacha`, which is the
chassis-mandated PRNG, which is already battle-tested."

**Gaussian transform.** The Langevin thermostat needs `N(0, 1)`
samples, not uniform bytes. The transform from uniform bytes to
Gaussian samples is Box-Muller applied to pairs of uniform
`f64`s drawn from the block. This is the same transform
`rand_distr::StandardNormal` uses internally (for one of its
algorithms), so there is no numerical-quality difference between
Route 2's Gaussian output and the current code's
`StandardNormal.sample(&mut *rng)` call at `langevin.rs:191`
(recon-reported). Box-Muller is ~5 lines on top of the PRF
primitive.

**Performance.** Each Langevin apply call needs `n_dofs`
Gaussian samples. One ChaCha8 block produces 64 bytes = 8 `u64`s
= 4 Box-Muller pairs = 8 Gaussians. For a 6-DOF Langevin
thermostat, one block per apply is sufficient; only the first 6
of the 8 Gaussians are used and the remaining 2 are discarded.
The per-apply cost is: one `chacha8_block` call (one ChaCha8
block computation) plus 6 Box-Muller transforms. The current
code's per-apply cost is: one `self.rng.lock()` call plus 6
`StandardNormal.sample` calls, each of which draws 2 `u64`s
from the ChaCha stream and applies Box-Muller internally — which
is structurally the same work as one ChaCha8 block plus 6
Box-Mullers, except the current code does the block computation
implicitly across the 12 `u64`s it consumes and the mutex lock
adds an uncontended acquisition/release. **No performance
regression expected.** (The wrapper in Section 4 shows the
single-block-per-apply pattern explicitly; a naive per-DOF
implementation would do 6 blocks per apply and discard 42 of
the 48 produced Gaussians, which is a 6× slowdown on the
hot path and should be avoided.)

**Route 2 is the pick.** Localized complexity, testable against
the chassis PRNG, no new dependencies, no mutex ceremony on the
component, honest about the primitive the thermostat crate is
taking on.

## Section 3 — Per-env seed derivation: not needed

The Phase 3 roadmap named "per-env seed derivation primitive" as
a Ch 15 design call. Under shape 1, this would be a real decision:
how do you derive a distinct RNG stream seed for each env from a
shared master seed? (Candidates: splitmix64, `SeedableRng::from_rng`,
hash-based mixing, or a manual XOR-and-multiply pattern.) Under
shape 3 the question dissolves.

Here is why. Shape 3's PRF takes `(master_seed, traj_id,
step_index, dof_idx)` as its coordinate. All four integers are
inputs to the same pure function. Each env's noise sequence is
distinct because each env has a distinct `traj_id`, not because
each env has a distinct derived seed. The "derivation" is done
implicitly by the PRF being keyed on `traj_id` as part of its
input; changing `traj_id` produces a different output at the same
`(step_index, dof_idx)` position.

**There is no per-env seed derivation primitive to specify.**
`traj_id` is literally the env index (0, 1, 2, ..., n_envs - 1),
set once by the `install_per_env` factory closure. The master
seed is a single `u64` held on each thermostat instance (set once
by the factory from an outer context). The PRF at `(master_seed,
traj_id, step_index, dof_idx)` is guaranteed to produce a
different sequence for different `traj_id` values, by the PRF's
construction — that is what it means to be a PRF keyed on its
inputs.

Under shape 1, by contrast, the question would have mattered. Each
env would need its own `ChaCha8Rng` stream, and the streams would
need to be seeded from non-overlapping initial seeds — which
would require a derivation function (splitmix64 or equivalent) to
turn a single master seed into N disjoint env seeds. Ch 14 Section
1 named this as a subclass of shape 1's design cost. Shape 3
eliminates it by construction.

**This is a side benefit of shape 3 that Section 1.1 did not rely
on but is worth naming explicitly.** Under shape 1, the per-env
seed derivation question would be a real design call that could
go wrong (e.g., the splitmix64 naive XOR with env index has a
collision mode at master seed 0, which a careless implementation
would miss). Under shape 3, the question does not arise and the
collision-mode category of bugs does not exist.

## Section 4 — Gating-trait preservation mechanism

The `Stochastic` sibling trait at
`sim/L0/thermostat/src/component.rs:101-108` (recon-reported) and
its FD invariant at `langevin.rs:311-363` (recon-reported) survive
the refactor **unchanged** in shape. The preservation mechanism
is:

1. The `stochastic_active: AtomicBool` field stays on each
   stochastic component. Under C-3, the Langevin thermostat's
   field list is `(gamma, k_b_t, master_key: [u8; 32], traj_id:
   u64, counter: AtomicU64, stochastic_active: AtomicBool)`,
   plus the existing `k_b_t_ctrl: Option<usize>` for runtime
   temperature modulation (unchanged from today). The flag is
   not moved to `EnvMeta`, is not replaced with a different
   gating primitive, and is not wrapped in any new type.
2. The `Stochastic` trait at `component.rs:101-108` is unchanged.
   `set_stochastic_active(&self, bool)` and
   `is_stochastic_active(&self) -> bool` remain the only methods.
3. `PassiveComponent::as_stochastic()` at `component.rs`
   (returning `Option<&dyn Stochastic>`) is unchanged. The
   thermostat's override at `langevin.rs:196-198` (recon-reported)
   is unchanged.
4. `PassiveStack::disable_stochastic` at `stack.rs:231`
   (recon-reported) and the `StochasticGuard` RAII type at
   `stack.rs:282` (recon-reported) are unchanged. The guard
   iterates stochastic components and flips their flags, and
   restores prior states on drop.
5. The `apply` method's early-return pattern at
   `langevin.rs:167-174` (recon-reported) is updated to guard
   the counter advance rather than the mutex lock. The updated
   structure, using the single-block-per-apply pattern from
   Section 2.3, is:

```rust
fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
    // Damping (deterministic half of the FD pair) runs
    // unconditionally.
    let h = model.timestep;
    let n_dofs = self.gamma.len();
    let k_b_t = self.effective_k_b_t(data);  // unchanged from today
    for i in 0..n_dofs {
        qfrc_out[i] += -self.gamma[i] * data.qvel[i];
    }

    // Gating check before the counter advance.
    if !self.stochastic_active.load(Ordering::Relaxed) {
        // No counter advance, no PRF call, no noise. Deterministic
        // forces are already in qfrc_out. FD invariant preserved.
        return;
    }

    // Counter advance + single PRF block for up to 8 DOFs.
    let step_index = self.counter.fetch_add(1, Ordering::Relaxed);
    let block_counter = encode_block_counter(self.traj_id, step_index);
    let block = chacha8_block(&self.master_key, block_counter);
    let gaussians = box_muller_from_block(&block);  // [f64; 8]

    for dof in 0..n_dofs {
        let gamma_i = self.gamma[dof];
        let sigma = (2.0 * gamma_i * k_b_t / h).sqrt();
        qfrc_out[dof] += sigma * gaussians[dof];
    }
}
```

The `effective_k_b_t` helper encapsulates the `k_b_t_ctrl`
runtime-temperature-modulation logic at `langevin.rs:154-156`
(recon-reported) — unchanged from today, just factored into a
method so the sketch stays focused on the PRF integration.
`box_muller_from_block` takes the 64-byte block, splits it into
4 `(u64, u64)` pairs, applies Box-Muller to each pair to produce
2 `f64` Gaussians, and returns the 8 Gaussians as a fixed-size
array. For `n_dofs > 8`, the method is extended to iterate the
block counter (one block per group of 8 DOFs); that extension
is a Part 4 concern and is dead code at the D2c rematch
configuration's 6-DOF SR particle.

**Constructor signature change.** The current
`LangevinThermostat::new(gamma, k_b_t, seed)` at `langevin.rs:102`
(recon-reported) grows to carry the master key and traj_id as
additional fields. The exact shape is a Part 4 PR concern with
two defensible options: (a) a direct constructor
`new(gamma, k_b_t, master_seed, traj_id)` that takes both as
arguments up front, or (b) a builder-style split
`new(gamma, k_b_t, master_seed)` followed by a
`with_traj_id(env_idx)` method called by the `install_per_env`
factory. The `seed` field on the struct, currently retained for
`diagnostic_summary` at `langevin.rs:97-100` (recon-reported), is
renamed to `master_seed` or `master_key` (whichever shape the
constructor commits to). The `diagnostic_summary` output at
`langevin.rs:212-232` (recon-reported) is updated to print
`(master_seed, traj_id, counter)` instead of the current `seed`-
only output. None of these changes affect the FD-invariant or
gating-trait contract; they are mechanical refinements to the
constructor surface.

**The FD invariant is preserved by the early-return.** When the
flag is inactive, `apply` executes only the damping loop and
exits before the `counter.fetch_add` call. The counter does not
advance. Post-re-enable, the next apply reads PRF at the same
`step_index` as it would have under no FD — and because the PRF
is a pure function, the noise sequence continues from exactly
where it was pre-disable. The post-FD noise is bit-identical to
the no-FD noise, which is the invariant the current code's test
`apply_does_not_advance_rng_when_stochastic_inactive` at
`langevin.rs:311-363` (recon-reported) operationalizes.

**The test at `langevin.rs:311-363` needs a small update.** Under
shape 1 the test compares `ChaCha8Rng` stream positions by reading
the first noise sample post-re-enable and comparing it to a fresh
thermostat's first sample. Under shape 3 the equivalent check is
to compare the first noise sample post-re-enable against the
PRF's output at the same `(master_seed, traj_id, step_index=0,
dof_idx=0)` coordinate as a fresh thermostat would produce. The
assertion shape is the same (two `f64` equality) but the
operational construction differs. Rewriting the test is part of
the Part 4 PR that lands C-3; the update is mechanical and does
not change what the test asserts.

**No new tests are needed for gating preservation specifically.**
The existing tests at `langevin.rs:241-266` (recon-reported) for
`new_initializes_stochastic_active_to_true`,
`set_stochastic_active_roundtrips`, and `as_stochastic_returns_some_self`
all remain valid. The `disable_stochastic` guard tests at
`stack.rs:366-446` (recon-reported), covering
`disable_stochastic_flips_only_stochastic_components_and_restores_on_drop`
and `disable_stochastic_preserves_already_disabled_components`,
are unchanged because the guard's contract (flip flags, restore
on drop) is unchanged.

## Section 5 — Regression test spec

Chapter 13 named a regression test as a Ch 15 requirement: *"a
test that installs a stochastic component, runs N parallel envs,
and asserts reproducibility across runs with the same master
seed. The sequential fallback is the oracle."* This section
specifies the test's shape, location, assertions, and pass
criterion.

### 5.1 What the test asserts

**The assertion.** After running N envs forward for K steps with
a `LangevinThermostat` installed via `install_per_env`, the
resulting `qpos` and `qvel` vectors for each env are bit-identical
between a sequential run and a parallel run at the same master
seed.

**Why bit-identical (not approximate).** Under shape 3 with the
Route 2 PRF, the noise at each env at each step is a pure
deterministic function of `(master_key, traj_id, step_index)`
(DOFs share one block per step). The function has no
order-dependence: calling it in any order produces the same
output at any given coordinate. This is the defining property
of a counter-based PRF and it is what makes shape 3 structurally
immune to the parallel-vs-sequential correctness bug Ch 10
argued about.

**The bit-identical claim rests on four independent properties,
and the claim earns the word "by construction" only when all
four hold.** (a) The PRF is a pure function of integers —
established in Section 2.3 and verified by the correctness test
against `rand_chacha`. (b) Each env's thermostat is a separate
instance with its own counter — established by hosting C and
the `install_per_env` factory. (c) `BatchSim::step_all`'s
sequential and parallel branches both step each env
independently, with no cross-env reductions in the step path —
this is a property of `BatchSim` as it exists today (each env
has its own `Data`, and the stepping code touches only that
env's `Data`), and it must continue to hold under the C-3
refactor for the regression test to pass. (d) Within one env,
the `cb_passive` closure is single-threaded under both branches
— this is already true today and `install_per_env` preserves it
by giving each env its own closure. Together, these four
properties imply that env *i*'s `(qpos, qvel)` trajectory is a
deterministic function of (initial state, master key,
traj_id = i), regardless of whether the outer loop over envs is
sequential or parallel. **If any future refactor introduces a
cross-env reduction in the step path** (a sum over envs before
the step is complete, a shared scratch buffer written from
multiple envs, any parallel-reduction-style operation) — the
bit-identical property needs to be re-checked, and the
regression test specified here becomes a reliable detector of
such regressions.

Given these four properties, the `(qpos, qvel)` trajectories
after K steps are bit-identical across parallel and sequential
runs at the same master seed. "Bit-identical" means `assert_eq!`
on the `DVector<f64>` fields, not `assert_relative_eq!`. Any
numerical difference between the two runs indicates either a
bug in the PRF purity or a new cross-env coupling in the step
path. Both are bugs this test is designed to catch.

### 5.2 Test location and crate

The test lives in `sim/L0/thermostat/tests/` as an integration
test (not a unit test in `src/`). The exact filename is a Part 4
PR concern; `parallel_matches_sequential_with_langevin.rs` or
`shape_3_parallel_reproducibility.rs` are both defensible. The
test depends on `sim_core::BatchSim`, `sim_thermostat::PassiveStack`,
`sim_thermostat::LangevinThermostat`, and `sim_mjcf::load_model`
— all already available to the thermostat crate's integration
test directory.

### 5.3 Test structure

The test's *structure* is what Ch 15 specifies: build two
batches from the same factory at the same master seed, run one
under the sequential path, run one under the parallel path,
compare `(qpos, qvel)` bit-for-bit across all envs. How the
sequential and parallel paths are selected is a Part 4 PR
concern — the current code's `BatchSim::step_all` is
feature-gated by the `parallel` Cargo feature, and two
defensible test strategies follow from that.

**Test strategy A (feature-gate-per-test).** Run the regression
test twice: once under `cargo test` (default features, no
`parallel`), which exercises the sequential path and writes the
result trace to a file; once under `cargo test --features
parallel`, which exercises the parallel path and compares its
result trace against the sequential file. This strategy makes
no assumptions about the shape of `BatchSim::step_all` and
works under the current feature-gate architecture without any
chassis-level API change.

**Test strategy B (in-process dual invocation).** If the C-3
refactor changes `BatchSim::step_all` to be runtime-selectable
(sequential vs parallel as arguments or as two separate methods
on `BatchSim`), the test runs in a single invocation, builds
two `BatchSim` handles from the same factory, invokes each
path on its handle, and compares traces in-memory. This is
cleaner to write but depends on an API change that Ch 15 does
not mandate.

Ch 15 recommends **strategy A for the initial landing**,
because it does not require deciding whether the feature-gate
architecture should change. A future refactor can adopt
strategy B by adding an in-process variant. A sketch of
strategy A (names and helpers are provisional; Part 4 picks
them precisely):

```rust
#[test]
fn parallel_matches_sequential_with_langevin_shape_3() {
    let master_seed = 0xD06F00D_42u64;
    let n_envs = 32;
    let n_steps = 1000;
    let k_b_t = 1.0;
    let gamma_val = 0.1;

    let factory = |env_idx: usize| {
        let model = sim_mjcf::load_model(FIXTURE_XML).unwrap();
        let thermostat = Arc::new(LangevinThermostat::new(
            DVector::from_element(model.nv, gamma_val),
            k_b_t,
            master_seed,
            env_idx as u64,
        ));
        let stack = PassiveStack::builder().with(thermostat).build();
        (model, stack)
    };

    // Use whichever BatchSim constructor the C-3 refactor
    // provides to take a factory closure. Under the feature
    // gate, `step_all` invokes the sequential path at default
    // features and the parallel path under `--features parallel`.
    let mut batch = build_batch_sim_per_env(n_envs, factory);
    for _ in 0..n_steps {
        batch.step_all();
    }

    // Dump (qpos, qvel) per env for comparison. Under strategy
    // A, the test serializes this to a file named by feature
    // flag (e.g. "trace-sequential.bin" or "trace-parallel.bin")
    // and the companion assertion runs after both variants have
    // produced their files.
    dump_env_state(&batch, trace_file_path_for_feature());
}

#[test]
fn parallel_and_sequential_traces_match() {
    // Loads the two trace files produced by the previous test
    // under both feature configurations, and asserts
    // bit-identical (qpos, qvel) across all envs.
    let seq = load_trace("trace-sequential.bin");
    let par = load_trace("trace-parallel.bin");
    assert_eq!(seq.len(), par.len());
    for (env_idx, (s, p)) in seq.iter().zip(par.iter()).enumerate() {
        assert_eq!(s.qpos, p.qpos, "env {env_idx} qpos divergence");
        assert_eq!(s.qvel, p.qvel, "env {env_idx} qvel divergence");
    }
}
```

Exact helper names (`build_batch_sim_per_env`,
`dump_env_state`, `load_trace`, `trace_file_path_for_feature`)
are provisional and settled during the Part 4 implementation.
The `BatchSim` constructor shape Ch 15 requires is: "takes a
factory closure of `FnMut(usize) -> (Model, Arc<PassiveStack>)`
and calls `install_per_env` internally" — the name and
signature are Part 4's call, but the shape is fixed by the
Section 1 pick of hosting C.

### 5.4 Seed coverage: one for correctness, three for robustness

Under shape 3, the PRF's purity means the correctness claim —
"parallel produces the same output as sequential" — is a
structural property of the implementation, not a statistical
property of a particular seed. A test that passes at one seed
passes at all seeds, for correctness purposes. Multi-seed
coverage adds no correctness evidence beyond what a single seed
already establishes.

A multi-seed sweep would add *robustness* evidence against
bugs in the seed-threading path (e.g., a bug where `traj_id` is
computed from `env_idx` incorrectly under some specific master
seed value). Running the test at two or three additional master
seeds (e.g., 0, `u64::MAX`, and a prime) would catch such bugs
and costs little. Ch 15 recommends running at **three** master
seeds: `0`, `0xD06F00D_42`, and `u64::MAX`. The first two exercise
common-case behavior; `u64::MAX` catches overflow bugs in the
counter encoding at high values. This is a test-parameterization
concern and can be done with `#[test]` × 3 or a single test with
a loop.

### 5.5 Why the sequential fallback is the oracle

Chapter 13 named the sequential fallback as "a ready-made
regression-test oracle" because it is bit-reproducible today and
stays bit-reproducible under any refactor that keeps the
sequential branch deterministic. Under C-3, the sequential
branch of `BatchSim::step_all` runs all envs on one thread, in
env index order. Each env's `cb_passive` closure is called in
order, and within each call the PRF is queried at the env's
current `(traj_id, step_index, dof_idx)` coordinate. The result
is deterministic in all inputs.

The parallel branch of `BatchSim::step_all` runs envs across
rayon's thread pool. Each env's `cb_passive` closure is still
single-threaded within its own env (because each env has its own
thermostat instance via `install_per_env`, with its own counter
and its own flag). The only difference from the sequential path
is scheduling — which environments happen to be processed at
which wall-clock time. Because the PRF is a pure function of
integers that do not depend on scheduling, the output is
bit-identical to the sequential path.

**The sequential-as-oracle approach is what makes the test
tractable.** Without an oracle, the test would need a pre-recorded
"expected noise tape" to compare against, or an external
reference implementation. With the oracle, the test compares
parallel against sequential at the same seed, and the sequential
path is self-evidently correct (single-threaded deterministic
integration) by the property Ch 13 named.

### 5.6 Existing test coverage and what changes

The current test at `sim/L0/thermostat/src/stack.rs` (the
`batch_matches_sequential_with_contacts` test Ch 11 cited) is
scoped to deterministic physics with contacts — no stochastic
component installed. That test remains valid under C-3: it covers
the "no stochastic component, parallel = sequential" case. The
new test Ch 15 specifies covers the "with a stochastic component
(Langevin), parallel = sequential" case, which is the case that
Chapter 13 named as the test-suite-level gap.

**The two tests together give full parallel-matches-sequential
coverage across the stochastic / non-stochastic boundary.** If
both pass, the chassis property "parallel and sequential produce
bit-identical state for the same master seed" holds under the C-3
refactor. If either fails, the refactor has a specific, localized
bug.

## What this chapter does not decide

It does not write the ChaCha8 PRF implementation — that is a
Part 4 PR concern, specifically the first PR in the Chassis
reproducibility sequence that Part 4 will lay out. It does not
specify the exact `BatchSim::new` constructor signature under the
C-3 rewire — the choice between "take a factory closure directly"
and "take a pre-built `EnvBatch` handle" is a Part 4 API-shaping
call, and both are equivalent in correctness. It does not
specify the exact `chacha8_block` function signature beyond the
sketch in Section 2.3; Part 4 refines it against the actual
thermostat crate's import surface.

It is worth being precise about what Ch 15 *does* decide
concerning a second stochastic component versus what it defers.
**Decided:** the convention any future stochastic component
should follow. Each new component takes `master_seed` and
`traj_id` as constructor arguments from its `install_per_env`
factory, carries its own `counter: AtomicU64` and `active:
AtomicBool`, and calls `chacha8_block` (or a thin wrapper) from
its `apply` method — with the gating early-return pattern
identical to Section 4's sketch. The convention is a
documented consequence of the C-3 pick and is not optional. It
is the shape a reviewer should expect when a new stochastic
component lands. **Deferred:** whether the convention should
be formalized as a trait (e.g., a `CounterBasedStochastic`
trait that encodes the shape at the type-system level). Ch 15
declines to formalize because introducing the trait before the
second stochastic component exists is speculative work against
a speculative future. The convention is good enough as
documentation until a second component forces the question,
and formalizing too early risks locking in a shape that turns
out to be wrong for the second component's specific needs.

It does not specify the test fixture for the regression test.
The test needs a small MJCF model with a Langevin thermostat
installed; the existing test fixtures in `sim/L0/thermostat/tests/`
include candidates (1-DOF SHO, multi-DOF particle). The Part 4
PR picks one.

It does not specify the landing strategy (single PR vs split).
Part 4's first chapter (the Chassis reproducibility PR plan) is
where the pieces get sequenced.

**What Ch 15 does decide** is the shape of the design: C-3 as the
cell, Route 2 as the PRF implementation, the master-seed-plus-
traj_id-on-component hosting pattern, the unchanged gating-trait
preservation, and the parallel-matches-sequential-with-Langevin
regression test. The Part 4 PR plans in the execution chapters
inherit this shape and turn it into landed code.
