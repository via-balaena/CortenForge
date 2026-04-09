# Thermo Crate Chassis Design

> **Status**: Active draft. One bolt pattern at a time.
> **Branch**: `feature/thermo-computing`
> **Owner**: Jon
> **Parent**: [MASTER_PLAN.md](./MASTER_PLAN.md)

This document defines the **bolt patterns** of the `sim-thermostat`
crate — the trait shapes, composition idioms, lifecycle handling, and
test infrastructure that components plug into. The implementation of
any specific component (`LangevinThermostat`, `BrownianRatchet`,
`StochasticDriver`, future thermostats) is **explicitly NOT** in this
document; that is component-spec content.

The R34 chassis principle: *components may be replaced wholesale, but
the bolt patterns must outlive every individual component*. The test
for whether something belongs in this document: if we threw away
`LangevinThermostat` next month and replaced it with `BAOABThermostat`,
this document should not need to change.

The reference architecture is `sim-ml-bridge`. Its modularity works
because the *traits* (`Environment`, `Algorithm`, `ObservationSpace`,
`ActionSpace`) were designed as the chassis, and the *implementations*
(CEM, TD3, SAC, PPO, REINFORCE) were designed as swappable components
against them. CEM can be replaced by TD3 by changing one type name.
This document does the same thing for the thermo crate.

---

## 0. Working principles

- **One bolt-pattern question at a time.** No batching.
- **Two schemes per decision**, compared honestly, recommended with
  reasoning. The user makes the final call.
- **The implementation may be wrong, that is OK** — as long as the
  bolt patterns are right. Phase 1 implementation is a learning
  opportunity for the chassis, not a commitment to never revise it.
- **Mechanical, predictable, swappable**. If the answer to "how do I
  add a new thermostat?" is anything other than "implement the trait,
  done," the chassis is wrong.

### Scope: this chassis is for passive forces only

*Added by doc review S4, 2026-04-09.*

This chassis is the bolt pattern for **passive forces only** —
things that fire inside `mj_fwd_passive()` via sim-core's
`cb_passive` callback and contribute to `qfrc_passive`. Controlled
forces (motor commands, RL action injection, manual loads) still
flow through sim-core's existing infrastructure: `cb_control` /
`ctrl` for actuator commands, `qfrc_applied` / `xfrc_applied` for
direct force injection. The thermo crate does not wrap, replace,
or extend any of those paths.

**Implication for components that need both** (e.g., D1's
flashing-ratchet Brownian motor, where an RL policy gates a
ratchet potential on/off): split the component into two halves
that share state via an `Arc`:

```rust
// The passive half — a PassiveComponent that bolts onto this chassis.
struct FlashingRatchetPotential {
    on: Arc<AtomicBool>,           // shared with the controller half
    bistable_params: ...,
}
impl PassiveComponent for FlashingRatchetPotential {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        if self.on.load(Ordering::Acquire) {
            // contribute the bistable potential gradient
        }
        // when off, contribute nothing
    }
}

// The controlled half — a cb_control callback (sim-core's existing hook).
let on = Arc::new(AtomicBool::new(false));
let on_for_passive = Arc::clone(&on);
let on_for_control = Arc::clone(&on);

PassiveStack::builder()
    .with(FlashingRatchetPotential { on: on_for_passive, ... })
    .build()
    .install(&mut model);

model.set_control_callback(move |m, d| {
    // policy decides whether to flip the ratchet at this step
    let flip = policy.decide(d);
    on_for_control.store(flip, Ordering::Release);
});
```

Both halves run in the same `step()` (control fires before
passive), share the `Arc<AtomicBool>` state, and compose without
either side knowing about the other. The chassis stays
single-purpose (passive forces); the cross-cutting active +
passive coordination is the user's composition of two existing
sim-core hooks. **D1 (Brownian motor), D2 (stochastic resonance
driver), and any future mixed active/passive component bolt onto
the chassis through this pattern.**

This is the only place the chassis acknowledges controlled forces.
Everywhere else in this document, "component" means
"`PassiveComponent` implementation." When the rest of the chassis
says "the only mutable surface is `qfrc_out`," that is a property
of the passive half — the controlled half is governed by sim-core's
own contracts on `cb_control` / `qfrc_applied`.

---

## 1. Decision Log

Append-only. Each decision documents the question, the schemes
considered, the recommendation, the user's call, and the reasoning.
Future-us must be able to reconstruct *why* the chassis looks the
way it does.

### Decision 1 (2026-04-09): The core trait shape

- **Question**: What does the core trait look like? Specifically:
  what does *every* thermo-crate component — `LangevinThermostat`,
  `BAOABThermostat`, `BrownianRatchet`, `StochasticDriver`, future
  unknowns — implement, and what does it require of the implementer?
- **Why this is first**: Every other bolt pattern (composition,
  clone handling, test utilities, crate layout) is downstream of
  the trait shape. Get this right and the rest falls into place.
  Get this wrong and the rest needs redoing.
- **Constraint from the recon round**: The chassis is `sim-core`'s
  `cb_passive` callback (item 3 part 4 + item 8 part 10). The
  callback signature is `Fn(&Model, &mut Data) + Send + Sync +
  'static`. Whatever trait we design, an instance must produce a
  closure of that shape that can be installed via
  `Model::set_passive_callback`.

#### Scheme A — Broad `PassiveComponent` trait

A minimal trait that abstracts "the thing that fires inside
`cb_passive`," with no physical interpretation baked in:

```rust
pub trait PassiveComponent: Send + Sync + 'static {
    /// Called once per `cb_passive` invocation. Reads from `model`
    /// and writes additive contributions into `data.qfrc_passive`.
    fn apply(&self, model: &Model, data: &mut Data);
}
```

`LangevinThermostat: PassiveComponent`. Also `BrownianRatchet:
PassiveComponent`. Also `StochasticDriver: PassiveComponent`. Also
`BAOABThermostat: PassiveComponent`. **All bolt onto the same
chassis with the same trait.**

Composition (Decision 2+) wraps a `Vec<Arc<dyn PassiveComponent>>`
in a single `cb_passive` closure that calls each component's
`apply` in order. Lifecycle (`install`, `clear`) lives on the
composer, not on each component — components are just "things
that get called."

Introspection (`temperature()`, `seed()`, etc.) is added by an
*orthogonal* trait if specific helpers need it:

```rust
pub trait Diagnose {
    fn diagnostic_temperature(&self) -> Option<f64> { None }
    fn diagnostic_seed(&self) -> Option<u64> { None }
    // ...
}
```

`LangevinThermostat: PassiveComponent + Diagnose`.
`BrownianRatchet: PassiveComponent` (Diagnose optional, not
required). A test helper that wants temperature requires
`T: PassiveComponent + Diagnose`.

**Pros**:
- Minimal surface, maximum reusability.
- Future-proof: anything that fires inside `cb_passive` (thermal
  noise, periodic forcing, ratchets, custom drag laws,
  energy-landscape biases) implements the same trait. D1 (Brownian
  motor), D2 (stochastic resonance driver), D3 (multi-component
  co-design), D5 (Brownian computer with multiple energy
  contributions) all bolt to the same chassis.
- The composer is one shape and works for any subtype mix.
- ml-bridge precedent: `Algorithm` is a broad trait abstracting
  "the thing you call to update a policy," not a thermostat-narrow
  concept.
- Matches the underlying mechanism: `cb_passive` is "a function
  called inside `mj_fwd_passive`." The trait abstracts exactly
  that — no physical interpretation imposed on top.
- Subtypes that need state (RNG, schedule, internal counters)
  manage it themselves; the trait does not impose a state model.

**Cons**:
- The trait is "callback-shaped" rather than "physically
  meaningful." A consumer of `dyn PassiveComponent` does not know
  whether it is a thermostat, a driver, or a ratchet — no
  type-level introspection without the orthogonal `Diagnose` trait.
- Helpers that want introspection need *two* trait bounds, not one.

#### Scheme B — Narrow `Thermostat` trait

A trait that *specifically* describes thermostats — bakes the
physical concepts (temperature, damping, seed) into the contract:

```rust
pub trait Thermostat: Send + Sync + 'static {
    fn apply(&self, model: &Model, data: &mut Data);
    fn temperature(&self) -> f64;        // kT
    fn damping(&self) -> &DVector<f64>;  // gamma vector
    fn rng_seed(&self) -> u64;
}
```

Drivers, ratchets, custom drag laws would need *separate* traits
or implement only a fallback `apply`-only trait.

**Pros**:
- Type-level distinction between thermostats and other passives.
- Introspection is built in: a generic test helper can ask
  `t.temperature()` without knowing the concrete type, with a
  single trait bound.
- Aligns with the physics: a thermostat *is* a thing with
  temperature + damping + seed.

**Cons**:
- **Narrower.** Drivers, ratchets, custom drag don't fit. We'd
  need a parallel hierarchy or a wider abstraction underneath
  anyway — at which point Scheme A's broader trait is the
  underlying thing and `Thermostat` is just a Scheme A
  subtype-with-extra-methods.
- Composition becomes tricky: a `Composer<Thermostat>` can compose
  thermostats, but mixing a thermostat with a driver requires the
  wider abstraction we've avoided.
- Premature: at Phase 1 we have ONE thermostat. Designing the trait
  around its specific shape is overfitting. If Phase 3 (D1 Brownian
  motor) needs a non-thermostat passive, we'd be retrofitting at
  exactly the moment the chassis matters most.
- Imposes a category boundary ("this is a thermostat, that is
  not") that the underlying chassis (`cb_passive`) does not draw.

#### Recommendation: **Scheme A — broad `PassiveComponent` trait**

Confidence: high. Five reasons in priority order:

1. **Future-proofness against the Research Directions**.
   `MASTER_PLAN.md` §2 names five concrete experiments. D1
   (Brownian ratchet/motor) and D2 (stochastic resonance with a
   sub-threshold periodic driver) are *not* thermostats but they
   are passive forces that bolt into `qfrc_passive` exactly the
   same way. Scheme A gives them a home with one trait. Scheme B
   forces a parallel hierarchy or a retrofit. Reach: Phase 3 — not
   far off.

2. **Match the chassis underneath**. The chassis is `cb_passive`,
   which is "a function called inside `mj_fwd_passive`." Scheme A
   abstracts exactly that mechanism. Scheme B layers physical
   interpretation on top of mechanism — and the moment we want
   non-thermostat passives, we have to peel the interpretation
   off again. Cleaner to never put it on.

3. **ml-bridge precedent.** ml-bridge's `Algorithm` trait is
   broad ("the thing you call to update a policy"), not narrow
   ("the thing that does CEM"). CEM, TD3, SAC, PPO, REINFORCE all
   bolt to it as siblings. The reference architecture the user
   explicitly named uses Scheme-A-shaped traits.

4. **Introspection is *additive*, not lost**. The orthogonal
   `Diagnose` trait recovers Scheme B's main advantage at the cost
   of one extra trait bound on the helpers that need it. Scheme A's
   trait stays minimal *and* the introspection-needing code paths
   work. Scheme B has no symmetric way to *remove* methods that
   non-thermostat passives don't have.

5. **Phase 1 has one component**. Designing the chassis around
   that one component's specific shape is the literal definition
   of overfitting. Scheme A is the smallest trait that lets Phase
   1 ship while remaining honest about Phase 3+'s needs.

**Why I'm not recommending Scheme B**: it solves the
introspection problem at a real cost (narrowing the trait), and
the introspection problem has a cheaper solution (the orthogonal
`Diagnose` trait). Scheme B is a local optimum that creates a
global problem by Phase 3.

#### Open follow-ons triggered by this decision

These do *not* need to be answered now — they are Decision 2+
material — but flagging them so the architecture document grows
in a known direction:

- **Decision 2**: How do components compose? `Composer<dyn
  PassiveComponent>` shape, builder pattern, install lifecycle.
- **Decision 3**: How is the clone footgun handled at the chassis
  level (Decision 1's trait does not yet say)?
- **Decision 4**: Where does `Diagnose` (and any other orthogonal
  traits) live and what methods does it expose at minimum?
- **Decision 5**: Public test utilities (`test_utils` module) —
  what assertion helpers does the chassis ship for any component
  to use?
- **Decision 6**: Crate layout — `lib.rs` exports, file
  organization, where component implementations live.

#### DECISION (user confirmed): **Scheme A — broad `PassiveComponent`**

Trait shape (revised by doc review M5 + M2, 2026-04-09 — see
"Trait contract revision" below and Decision 7):

```rust
pub trait PassiveComponent: Send + Sync + 'static {
    /// Add this component's per-DOF passive force contribution to
    /// `qfrc_out`. The component MUST NOT mutate `data` — it is passed
    /// immutably so the component can read state (`qpos`, `qvel`,
    /// `model.timestep`, etc.) without being able to write to it.
    /// `qfrc_out` is the only legal write target; it has length
    /// `model.nv` and is shared (accumulating) with other components
    /// in the same `PassiveStack`.
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>);

    /// If this component has a stochastic part, return a handle to
    /// toggle it. Default: no stochastic part, returns `None`.
    /// Components that implement `Stochastic` (Decision 7) override
    /// this to return `Some(self)`.
    fn as_stochastic(&self) -> Option<&dyn Stochastic> { None }
}
```

Introspection lives on the orthogonal `Diagnose` trait
(Decision 4). All future thermo-crate components — thermostats,
drivers, ratchets, custom drag laws, energy biases — implement
`PassiveComponent` and bolt to the same chassis.

##### Trait contract revision (doc review M5, 2026-04-09)

The original Decision 1 trait gave each component `&mut Data` —
exactly what `cb_passive` natively provides. That preserved
maximum flexibility but enforced *nothing*: a buggy or hostile
component could legally mutate `qpos`, `qvel`, `qfrc_actuator`,
`time`, or any other field, breaking physics in ways that take
days to debug. The contract was implied by prose, not by the
type system.

The revised trait moves the only mutable surface to a `&mut
DVector<f64>` (the per-DOF force contribution) and demotes
`Data` to immutable. Three consequences:

1. **Type-enforced contract.** A component physically cannot
   write to anything except its `qfrc_out` buffer. The class of
   bugs M5 was opened to address becomes uncompilable.
2. **Read-only access to `Data` is preserved.** Components still
   read `data.qvel`, `data.qpos`, `data.time`, `model.timestep`
   etc. through the immutable `&Data` and `&Model` references —
   everything legitimate the previous signature allowed.
3. **`PassiveStack::install` allocates a scratch buffer per
   step**, runs each component into it (accumulating across the
   stack), then writes `data.qfrc_passive += scratch` once at the
   end. Allocation cost is one `DVector::zeros(nv)` per step —
   tens of bytes at Phase 1 (`nv=1`) up to a few KB at the
   largest plausible Phase 5+ scale (`nv ≈ 1000`); negligible
   compared to one constraint solve. See Decision 2 for the
   updated install body.

Why this is the right call (consistent with the "loud over
silent" line drawn in items 2, 4, and 6 of the master plan
recon): every other "type-enforced vs. doc-trust" decision in
this initiative has gone the same way. Item 2 chose sole-writer
overwrite over additive (loud failure beats silent compounding).
Item 4 chose `ChaCha8Rng` over `StdRng` (loud reproducibility
beats silent algorithm-bump drift). Item 6 chose read-fresh over
cache (loud robustness beats silent precompute hazard). M5
applies the same principle to the trait contract itself.

The breaking change is free *right now* because no code has
been written. After Phase 1 ships, breaking the trait would
ripple through every component implementation; before Phase 1
ships, it's a one-line edit per (zero) implementations.

#### Status

- **Decision 1 RESOLVED.** Scheme A confirmed. The trait shape
  above is the chassis for every component the thermo crate ever
  ships.
- **Trait signature revised by doc review M5 (2026-04-09)** —
  `apply` now takes `&Data` and `&mut DVector<f64>` instead of
  `&mut Data`. The "broad trait" decision itself is unchanged;
  only the write-target contract is tightened.
- No code, no Cargo.toml changes, no new files written beyond this
  document.

---

### Decision 2 (2026-04-09): Composition idiom

- **Question**: How do `PassiveComponent` implementations compose?
  `sim-core`'s `cb_passive` is single-slot — only one callback per
  Model. If a user wants Langevin thermostat + sub-threshold
  periodic driver + Brownian ratchet running together (D1, D2, D3
  territory), the chassis needs a way to take N components and
  install ONE callback that calls them all in order. What does the
  composer look like, what does its install API look like, and what
  is the user-facing call shape?
- **Why this is decision 2**: Decision 1 settled the trait. Decision
  2 settles how multiple instances of the trait coexist. Everything
  about the lifecycle (install, clear), test-utility shape, and
  clone handling depends on the composer existing first.
- **Constraint from the chassis**: `Model::set_passive_callback`
  takes a single `Fn(&Model, &mut Data) + Send + Sync + 'static`.
  Whatever composer we design must produce exactly one such closure
  internally and install it via this API. Verified `pub` in
  Decision 8 / part 10 of the master plan recon.

#### Scheme A — Mutable vec with explicit push

A plain stack with a `new`/`push`/`install` lifecycle:

```rust
pub struct PassiveStack {
    components: Vec<Arc<dyn PassiveComponent>>,
}

impl PassiveStack {
    pub fn new() -> Self { Self { components: Vec::new() } }
    pub fn push<C: PassiveComponent>(&mut self, component: C) {
        self.components.push(Arc::new(component));
    }
    pub fn install(self: Arc<Self>, model: &mut Model) { ... }
}
```

User code:

```rust
let stack = Arc::new({
    let mut s = PassiveStack::new();
    s.push(LangevinThermostat::new(gamma, kT, seed));
    s.push(StochasticDriver::new(...));
    s
});
stack.install(&mut model);
```

**Pros**:
- Direct, mechanical, no extra builder type.
- One type to learn (`PassiveStack`) instead of two
  (`PassiveStack` + `PassiveStackBuilder`).

**Cons**:
- Two-step pattern: build the stack mutably, *then* wrap in `Arc`,
  *then* install. The `Arc::new({ let mut s = ...; s })` dance is
  awkward and is exactly the kind of pattern beginners trip on.
- `let mut` is required, breaking the chained-fluent style.
- Diverges from `sim-ml-bridge`'s `ActionSpace::builder()` idiom,
  which is the explicit reference architecture for the thermo
  crate. The user has to learn two patterns: builder for
  ml-bridge, vec-and-push for thermo.
- Single-component case is just as awkward as multi-component case
  — the boilerplate cost is uniform, no advantage when N=1.

#### Scheme B — Builder pattern (matches `ActionSpace::builder`)

A builder type that accumulates components fluently and `build()`s
into an `Arc<PassiveStack>` ready to install:

```rust
pub struct PassiveStack {
    components: Vec<Arc<dyn PassiveComponent>>,
}

pub struct PassiveStackBuilder {
    components: Vec<Arc<dyn PassiveComponent>>,
}

impl PassiveStack {
    pub fn builder() -> PassiveStackBuilder {
        PassiveStackBuilder { components: Vec::new() }
    }
}

impl PassiveStackBuilder {
    pub fn with<C: PassiveComponent>(mut self, component: C) -> Self {
        self.components.push(Arc::new(component));
        self
    }
    pub fn build(self) -> Arc<PassiveStack> {
        Arc::new(PassiveStack { components: self.components })
    }
}

impl PassiveStack {
    pub fn install(self: Arc<Self>, model: &mut Model) {
        let me = Arc::clone(&self);
        model.set_passive_callback(move |m, d| {
            // M5: components write into a scratch buffer, not into d
            // directly. The stack folds the result into qfrc_passive
            // after every component has run.
            let mut scratch = DVector::zeros(d.qfrc_passive.len());
            for c in &me.components {
                c.apply(m, d, &mut scratch);
            }
            for i in 0..scratch.len() {
                d.qfrc_passive[i] += scratch[i];
            }
        });
    }
}
```

User code (single component):

```rust
PassiveStack::builder()
    .with(LangevinThermostat::new(gamma, kT, seed))
    .build()
    .install(&mut model);
```

User code (multi-component):

```rust
PassiveStack::builder()
    .with(LangevinThermostat::new(gamma, kT, seed))
    .with(StochasticDriver::new(...))
    .build()
    .install(&mut model);
```

`with` takes `impl PassiveComponent` so the user does NOT have to
write `Arc::new(...)` at every call site — the builder wraps
internally. The user gets clean, chained, mechanical syntax for
both single and multi cases with no `let mut`.

**Clear path**: not on the stack at all. The user calls
`model.clear_passive_callback()` directly — that is already a
public sim-core API and there is no value in wrapping it. The
thermo crate does not duplicate sim-core APIs unnecessarily.

**Pros**:
- **One pattern for all cases** — single-component (N=1) and
  multi-component (N≥2) use the exact same call shape. Beginners
  learn one thing, and adding a component later is a one-line
  change (`.with(...)`).
- **Matches `sim-ml-bridge::ActionSpace::builder()` exactly** — the
  user already knows this pattern from ml-bridge. The thermo crate
  reads as a sibling rather than as a different shape.
- **`with(impl PassiveComponent)` is ergonomic** — no `Arc::new`
  noise at the call site, the builder wraps internally.
- **Chained fluent style** — no `let mut`, no `Arc::new({ ... })`
  dance. Reads top-to-bottom like a sentence.
- **Build returns the install-ready Arc** — one less ceremony
  step for the user.

**Cons**:
- Two types instead of one (`PassiveStack` + `PassiveStackBuilder`).
  Trivial cost: ~30 LOC of crate code, zero user-facing cost.
- Slight overhead in the trivial single-component case
  (`.builder().with(...).build()` vs. one constructor call). Real
  but ~5 chars; outweighed by the consistency benefit.

#### Recommendation: **Scheme B — builder pattern**

Confidence: high. Four reasons in priority order:

1. **ml-bridge precedent is the explicit reference architecture
   for this crate.** The user named ml-bridge's modularity as the
   model when reframing the chassis design. `ActionSpace::builder()`
   is exactly this pattern. Diverging from it would require
   beginners to learn two builder shapes for two L0 crates that
   are conceptually parallel — direct violation of "readability
   and organization are the highest priority."

2. **One pattern for single and multi cases**. The trivial case
   (one thermostat) and the headline case (D3 multi-component
   co-design with thermostat + driver + ratchet) use *the same
   API shape*. Beginners who start with one component and later
   add a second do not have to relearn anything; they add one
   `.with(...)` line. This is the mechanical-and-modular property
   the user explicitly asked for.

3. **No `Arc::new` noise at the call site**. `with(impl
   PassiveComponent)` taking the concrete type and wrapping
   internally eliminates a category of beginner errors (forgetting
   to wrap, double-wrapping, wrapping the wrong thing). The
   builder owns the wrapping discipline.

4. **The cost is invisible to users**. Two types in the crate
   (`PassiveStack` + `PassiveStackBuilder`) are implementation
   details. The user-facing surface is one fluent call chain.

**Why I'm not recommending A**: Scheme A's "one type" advantage is
real but its `Arc::new({ let mut s = ...; s })` dance is the
exact pattern that drives beginners away from Rust crates. Scheme
B trades one extra type for vastly better ergonomics. The trade
is uneven in B's favor.

#### Sub-decisions inside Decision 2

Locking these in alongside the scheme so the API surface is clear:

- **Builder method name**: `with(component)`. Reads as "the stack
  with this component"; matches Rust's `with_*` builder convention
  (cf. `nalgebra::Matrix::with_diagonal`,
  `bevy::Transform::with_translation`, etc.). Considered: `add`,
  `push`, `component`. `with` is the most idiomatic; the others
  read as imperative or noisy.
- **`with` signature**: `with<C: PassiveComponent>(mut self,
  component: C) -> Self`. Takes the concrete type by value,
  wraps in `Arc` internally. Considered: `with(Arc<dyn
  PassiveComponent>)` (more flexible but requires user to wrap)
  and `with(Arc<impl PassiveComponent>)` (middle ground). The
  concrete-type form is the most ergonomic for Phase 1; if a
  later phase needs to share an `Arc<dyn PassiveComponent>`
  between stacks, we can add a parallel `with_arc` method
  without breaking `with`.
- **`build` return type**: `Arc<PassiveStack>`. Pre-wrapped so
  the user does not have to think about `Arc`. The `install`
  method takes `self: Arc<Self>` so the chained-fluent style
  works: `.build().install(&mut model)`.
- **Order semantics**: Vec order = call order. The first
  `.with(...)` call's component runs first inside the
  `cb_passive` closure. This is the obvious behavior; document
  it explicitly so users can rely on it (e.g., for stochastic
  driver-then-thermostat sequencing).
- **`clear` is NOT on `PassiveStack`**. Users call
  `model.clear_passive_callback()` directly — that is already a
  public sim-core API. Don't duplicate.
- **Component inspection (`.components() -> &[...]`) is NOT in
  Phase 1**. Not needed yet. Add when there is a concrete use
  case (probably Decision 4 or Decision 5).

#### Open follow-ons triggered by this decision

- The `install` method takes `self: Arc<Self>`, which means a
  user who clones the underlying `Arc` and installs it on two
  Models gets *one shared stack*, *one shared component set*,
  *one shared RNG state per stateful component*. **This is the
  clone footgun.** Decision 3 resolves it at the chassis level
  with a "build N independent stacks from one prototype"
  pattern, so Decision 2 does not need to solve it — but the
  install signature here makes Decision 3 inescapable.

#### DECISION (user confirmed): **Scheme B — builder pattern**

Final API surface:

```rust
pub struct PassiveStack {
    components: Vec<Arc<dyn PassiveComponent>>,
}

pub struct PassiveStackBuilder {
    components: Vec<Arc<dyn PassiveComponent>>,
}

impl PassiveStack {
    pub fn builder() -> PassiveStackBuilder { ... }

    /// Install the stack as the model's `cb_passive` callback.
    /// Each step, allocates a scratch `DVector::zeros(model.nv)`,
    /// runs every component into it (accumulating across the stack),
    /// then folds the result into `data.qfrc_passive`. Components
    /// see `&Data` and `&mut DVector<f64>` only — they cannot mutate
    /// any other field by construction (M5).
    pub fn install(self: Arc<Self>, model: &mut Model) {
        let me = Arc::clone(&self);
        model.set_passive_callback(move |m, d| {
            let mut scratch = DVector::zeros(d.qfrc_passive.len());
            for c in &me.components {
                c.apply(m, d, &mut scratch);
            }
            for i in 0..scratch.len() {
                d.qfrc_passive[i] += scratch[i];
            }
        });
    }
}

impl PassiveStackBuilder {
    pub fn with<C: PassiveComponent>(mut self, component: C) -> Self {
        self.components.push(Arc::new(component));
        self
    }
    pub fn build(self) -> Arc<PassiveStack> {
        Arc::new(PassiveStack { components: self.components })
    }
}
```

User code is uniform across single-component and multi-component
cases:

```rust
PassiveStack::builder()
    .with(LangevinThermostat::new(gamma, kT, seed))
    .with(StochasticDriver::new(...))
    .build()
    .install(&mut model);
```

#### Status

- **Decision 2 RESOLVED.** Scheme B confirmed. The composer is
  the builder-pattern `PassiveStack` + `PassiveStackBuilder`,
  matching `sim-ml-bridge::ActionSpace::builder()`. This is the
  one and only composition path for the thermo crate.
- No code, no Cargo.toml changes, no new files written beyond
  this document.

---

### Decision 3 (2026-04-09): Clone footgun resolution

- **Question**: When a user installs a `PassiveStack` on a
  `Model` and then clones the `Model` for parallel envs, the
  cloned Model inherits the *same* `Arc<dyn Fn...>` callback —
  pointer-copied — which means it shares the captured
  `Arc<PassiveStack>`, which means it shares every stateful
  component inside (most importantly, every `Mutex<RNG>`).
  Result: interleaved noise streams across envs (wrong physics)
  and lock contention (perf disaster). What does the thermo
  crate provide so this footgun is hard to trigger?
- **Why this is decision 3**: Decisions 1 and 2 settled the
  trait and the composer; Decision 2's `install` signature
  takes `self: Arc<Self>`, which makes the footgun *physically
  inescapable* without an explicit API response. We owe the
  user one.
- **What we cannot change**: `sim-core`'s
  `Model::set_passive_callback` wraps the closure in
  `Callback(Arc<dyn Fn...>)` internally
  (`sim/L0/core/src/types/model.rs:1242`). This is verified
  shipped behavior and modifying it is out of scope for the
  thermo crate (item 8 / part 10 of the recon log explicitly
  required zero sim-core changes; the entire sibling-crate
  decision rests on this).
- **What we cannot achieve**: A *fully loud* solution at the
  type level. The fundamental cause is that `Model::clone()` is
  a sim-core operation that the thermo crate cannot intercept
  or override. **Every option below is a degree of "make the
  right thing easy and the wrong thing harder," not "make the
  wrong thing impossible."** This is unavoidable given the
  chassis constraint, but it must be acknowledged honestly.

#### Scheme A — Documentation only

Provide no new API. Document loudly in the rustdoc of
`PassiveStack::install` and in the crate README that:

> If you intend to use this `Model` with `BatchSim` or otherwise
> clone it, **install the stack AFTER cloning**, not before. The
> passive callback is `Arc`-shared across clones; installing
> before cloning means every parallel env shares the same RNG
> mutex, producing interleaved noise streams (wrong physics)
> and lock contention.

User pattern (correct):
```rust
let prototype = sim_mjcf::load_model(xml)?;
let envs: Vec<Model> = (0..N).map(|i| {
    let mut model = prototype.clone();  // clone first, no callback
    PassiveStack::builder()
        .with(LangevinThermostat::new(gamma.clone(), kT, seed_base + i))
        .build()
        .install(&mut model);            // install after
    model
}).collect();
```

**Pros**:
- Zero new API surface.
- The user has full control.
- The right pattern is straightforward once known.

**Cons**:
- **Silent footgun if the order is wrong**. Install-then-clone
  is the natural mental model for many users (build everything
  first, clone for parallelism second). Documentation alone is
  exactly the failure mode sharpen-the-axe forbids — silent,
  delayed, hard to debug.
- Beginners and Claude-equivalents reading the codebase are not
  guaranteed to read every rustdoc carefully before writing
  their first batch-sim setup.

#### Scheme B — `install_per_env` factory + defensive clear

Provide a single focused helper on `PassiveStack` that takes a
prototype, an env count, and a per-env stack-builder closure,
and produces N independent `(Model, fresh callback)` pairs.
**Defensively clears the prototype's callback on each clone**
before installing the fresh stack.

```rust
impl PassiveStack {
    /// Build `n` independent envs from a prototype Model.
    ///
    /// For each env `i`, this:
    /// 1. Clones the prototype.
    /// 2. **Clears any passive callback on the clone** (defensive
    ///    — guarantees no shared state inherited from the
    ///    prototype, even if the prototype was previously
    ///    installed on).
    /// 3. Calls `build_one(i)` to construct a fresh
    ///    `Arc<PassiveStack>` for env `i`.
    /// 4. Installs that fresh stack on the cloned model.
    ///
    /// Returns an `EnvBatch` containing N independent Models AND
    /// the per-env stacks (S3 — doc review 2026-04-09): the common
    /// case unwraps `envs.models`, but the introspection case can
    /// reach `envs.stacks[i].diagnostic_summary()` without having
    /// to recreate the stacks.
    ///
    /// **Use this for BatchSim setups.** Naïvely cloning a
    /// Model after installing a stack will share the underlying
    /// callback Arc across clones — see the warning on
    /// [`PassiveStack::install`].
    pub fn install_per_env<F>(
        prototype: &Model,
        n: usize,
        build_one: F,
    ) -> EnvBatch
    where
        F: Fn(usize) -> Arc<PassiveStack>,
    {
        let mut models = Vec::with_capacity(n);
        let mut stacks = Vec::with_capacity(n);
        for i in 0..n {
            let stack = build_one(i);
            let mut model = prototype.clone();
            model.clear_passive_callback();  // defensive
            stack.clone().install(&mut model);
            models.push(model);
            stacks.push(stack);
        }
        EnvBatch { models, stacks }
    }
}

/// Result of `PassiveStack::install_per_env`. Holds N independent
/// Models alongside the per-env stacks that produced them.
///
/// Common case: take the models with `envs.models` (a one-token
/// unwrap; the field is `pub`).
///
/// Introspection case: reach into `envs.stacks[i]` to call
/// diagnostic accessors, extract per-env RNG state for
/// checkpointing, etc.
pub struct EnvBatch {
    pub models: Vec<Model>,
    pub stacks: Vec<Arc<PassiveStack>>,
}
```

User code (BatchSim setup with N envs, each with its own
seed):
```rust
let envs = PassiveStack::install_per_env(
    &prototype,
    N,
    |i| {
        PassiveStack::builder()
            .with(LangevinThermostat::new(
                gamma.clone(),
                kT,
                seed_base + i as u64,
            ))
            .build()
    },
);
// envs.models is Vec<Model>, ready for BatchSim::new(envs.models)
// envs.stacks[i] is the per-env Arc<PassiveStack> for introspection
```

The closure takes `i` (the env index) so users can vary
parameters per env (parameter sweeps, varying initial
conditions, varying seeds). Seed derivation is the user's
concern — the chassis doesn't impose a particular scheme.

**Plus**: the rustdoc for `PassiveStack::install` (the regular
single-env install) carries the same warning as Scheme A's
documentation, pointing users to `install_per_env` for the
batch case. So Scheme B *includes* Scheme A's documentation
discipline — it adds an API on top of it, not as a replacement.

**Pros**:
- **Loud at the API level for the case that matters**. The
  BatchSim case is the *only* case where the footgun is
  triggerable (single-env users can't clone, so they can't
  hit it). Scheme B provides exactly one API for that case
  and documents it as THE batch-sim entry point.
- **Defensive `clear_passive_callback` makes the API
  bulletproof against the install-then-batch order**. Even
  if the user installs the stack on the prototype first and
  *then* calls `install_per_env`, the resulting envs are
  still independent because each clone has its callback
  cleared inside the loop. **The order of install vs
  install_per_env doesn't matter** — install_per_env always
  produces correct results.
- **The closure-builder pattern matches Decision 2's
  builder-pattern idiom**. The user is already mentally in
  "build a stack with `.with(...).build()`" mode; the
  closure is just "do that build, but per env."
- **Doesn't require sim-core changes**. Honors item 8's
  "zero sim-core changes" constraint.
- **Doesn't add complexity to the trivial case**. Single-env
  users use `builder().with(...).build().install(&mut
  model)` exactly as Decision 2 shipped it. `install_per_env`
  exists only for batch users.

**Cons**:
- Adds ~40 LOC of API surface to the chassis.
- Doesn't *prevent* a user from doing
  `model.clone()` → install → `model.clone()` again. The user
  can still write the wrong code if they ignore the API. The
  guard rail is "the right thing is one named API call;"
  not "the wrong thing is impossible."
- The closure-taking-an-index pattern is slightly less
  beginner-friendly than `builder().with(...).build()`. But
  the BatchSim use case is inherently more advanced anyway.

#### Schemes considered and rejected

- **Scheme C — `ThermoModel` wrapper type**: wrap `Model` in a
  thermo-crate type that prevents naïve cloning. **Rejected**
  because it fragments the ecosystem (every other sim-core API
  works on `Model` directly), users have to learn a parallel
  type, and ml-bridge specifically chose *not* to wrap `Model`
  for the same reasons.
- **Scheme D — Modify `sim-core::set_passive_callback` to take
  a factory closure** (`Fn() -> impl Fn(...)` so a fresh
  closure is produced per clone). **Rejected** because it
  requires modifying sim-core's shipped callback infrastructure,
  breaks the 3 existing in-tree `cb_passive` consumers in
  `tests/integration/callbacks.rs`, and explicitly violates
  item 8 / part 10's "zero sim-core changes" constraint that
  the entire sibling-crate decision rests on. Worth flagging
  as the *eventual* right answer if a future user demands
  truly-impossible-to-misuse semantics, but Phase-1-out-of-
  scope.
- **Scheme E — Type-state on `PassiveStack`** (e.g.,
  `InstalledPassiveStack` after install): doesn't propagate to
  `Model::clone()`, which is where the actual sharing happens.
  **Rejected** as not addressing the root cause.

#### Recommendation: **Scheme B — `install_per_env` factory + defensive clear + warnings on `install`**

Confidence: high. Five reasons in priority order:

1. **Bulletproof against the actual footgun**. The
   `clear_passive_callback` inside the loop guarantees per-env
   independence regardless of what state the prototype is in.
   A user who calls `install_per_env(&prototype, ...)` cannot
   accidentally inherit shared state from the prototype, even
   if the prototype was installed on. **This is the closest
   thing to "loud at the API level" we can get without
   modifying sim-core.**

2. **Doesn't burden the trivial case**. Single-env users use
   the Decision 2 builder pattern unchanged. Only BatchSim
   users encounter the new helper, and the BatchSim case is
   inherently more advanced.

3. **The closure-builder mirror of Decision 2's builder
   pattern is consistent**. `install_per_env(&prototype, n,
   |i| PassiveStack::builder().with(...).build())` reads as
   "do that builder thing, per env." The user is not learning
   a new mental model — they're applying the existing one
   inside a `for i in 0..n` loop the chassis manages.

4. **Honors item 8's chassis constraint**. Zero sim-core
   changes. The thermo crate stays a clean bolt-on.

5. **Not the *true* solution, but the *honest* one**. The
   true solution requires either modifying sim-core (Scheme D,
   out of scope) or redesigning `Model::clone()` itself
   (impossible). Scheme B is the best the chassis can offer
   given the constraint, and naming it explicitly — "this is
   loud at the API level, not at the type level, and here's
   why" — preserves the integrity of the sharpen-the-axe
   discipline more than pretending we have a fully loud
   answer would.

#### Sub-decisions inside Decision 3

- **Function name**: `install_per_env`. Reads as "install one
  stack per env, fresh each time." Considered: `install_batch`
  (ambiguous w.r.t. sim-core's `BatchSim`), `install_n`
  (uninformative), `install_independent` (verbose), `replicate`
  (too short). `install_per_env` is the clearest match for the
  BatchSim mental model.
- **Return type**: `EnvBatch { pub models: Vec<Model>, pub
  stacks: Vec<Arc<PassiveStack>> }` (revised by doc review S3,
  2026-04-09). Originally `Vec<Model>` — that lost access to the
  per-env stacks for any introspection use case (diagnostic
  summaries, per-env RNG checkpointing, parameter inspection),
  forcing the user to keep the stacks alive in the closure's
  outer scope by hand. The `EnvBatch` form gives the common case
  a one-token unwrap (`envs.models`) and the introspection case
  works for free (`envs.stacks[i]`). Cost: one struct definition
  with two `pub` fields (~10 LOC). Considered: a tuple
  `(Vec<Model>, Vec<Arc<PassiveStack>>)` — rejected because
  named fields are clearer at the call site (`envs.models`
  vs `envs.0`); a single `Vec<(Model, Arc<PassiveStack>)>` —
  rejected because the common case wants two separate Vecs
  (`BatchSim::new(envs.models)` is one move, not a per-element
  destructure).
- **Closure signature**: `Fn(usize) -> Arc<PassiveStack>`. The
  index lets users vary parameters per env (parameter sweeps,
  varying ICs, varying seeds). Considered: `Fn() ->
  Arc<PassiveStack>` (no index — but then per-env variation
  becomes external bookkeeping); `Fn(usize, u64) -> ...` (with
  derived seed — rejected as imposing a seeding scheme on the
  user, when seed derivation is the user's concern).
- **Defensive clear is non-negotiable**. The
  `model.clear_passive_callback()` inside the loop is what
  makes Scheme B bulletproof against the install-then-batch
  order. Document it explicitly in the rustdoc.
- **Warning on `install`'s rustdoc**: required. The single-env
  `install` method must warn about the clone footgun and point
  users to `install_per_env` for batch use. This is the
  Scheme A documentation discipline applied alongside Scheme B.
- **No `clone_thermo_model` standalone helper**. Considered as
  a third API, rejected because `install_per_env` already
  covers the case (a user wanting one independent clone calls
  `install_per_env(&prototype, 1, |_| ...)` and takes the
  first element). Adding a parallel single-clone helper would
  fragment the API.

#### DECISION (user confirmed): **Scheme B — `install_per_env` factory + defensive clear + warnings on `install`**

The chassis explicitly accepts a "loud at the API level, not at
the type level" trade-off, with the understanding that this is
the best the sibling-crate constraint from item 8 permits.
Scheme D (modify sim-core) is named as the *eventual* right
answer if a future user demands truly impossible-to-misuse
semantics, but is Phase-1-out-of-scope.

Final API surface (return type revised by doc review S3,
2026-04-09 — `Vec<Model>` → `EnvBatch`):

```rust
pub struct EnvBatch {
    pub models: Vec<Model>,
    pub stacks: Vec<Arc<PassiveStack>>,
}

impl PassiveStack {
    pub fn install_per_env<F>(
        prototype: &Model,
        n: usize,
        build_one: F,
    ) -> EnvBatch
    where
        F: Fn(usize) -> Arc<PassiveStack>,
    {
        let mut models = Vec::with_capacity(n);
        let mut stacks = Vec::with_capacity(n);
        for i in 0..n {
            let stack = build_one(i);
            let mut model = prototype.clone();
            model.clear_passive_callback();  // defensive — non-negotiable
            stack.clone().install(&mut model);
            models.push(model);
            stacks.push(stack);
        }
        EnvBatch { models, stacks }
    }
}
```

Plus a loud warning in the rustdoc of `PassiveStack::install`
pointing users to `install_per_env` for batch use.

#### Status

- **Decision 3 RESOLVED.** Scheme B confirmed. The chassis
  provides one named API for the BatchSim case, defensively
  clears the inherited callback inside the loop, and documents
  the trap on the regular `install` method.
- **Return type revised by doc review S3 (2026-04-09)**:
  `Vec<Model>` → `EnvBatch { models, stacks }`. Common case
  is unchanged ergonomically (`envs.models` instead of bare
  `envs`); introspection case (per-env diagnostic summaries,
  RNG checkpointing, parameter inspection) now works without
  external bookkeeping. Adds ~10 LOC (one struct definition).
- The Scheme D trapdoor is named in the document so future-us
  has a clear path if the constraint ever changes.
- No code, no Cargo.toml changes, no new files written beyond
  this document.

---

### Decision 4 (2026-04-09): `Diagnose` trait surface

- **Question**: What does the `Diagnose` trait — promised in
  Decision 1 as "the orthogonal home for introspection" — look
  like at Phase 1? Specifically: (a) do we ship it at all in
  Phase 1, or defer until a real consumer appears? (b) if we
  ship it, what methods does it expose at minimum?
- **Why this is decision 4**: Decision 1 promised that
  introspection has a home if needed; that promise needs to be
  cashed out (or deferred deliberately) before the Phase 1 spec
  can describe how tests interact with the components.
- **Constraint from Decision 1**: `Diagnose` must be *orthogonal*
  to `PassiveComponent`. Components opt in by implementing it;
  components that don't have meaningful introspection
  (e.g., a stateless driver) just don't implement it. The two
  traits compose via trait bounds (`T: PassiveComponent + Diagnose`)
  on consumers that need both.

#### Who actually needs introspection in Phase 1?

Walking through the planned Phase 1 surface:

- **Equipartition test**: Tests the *concrete* `LangevinThermostat`
  type. Uses concrete-type accessors directly (`thermostat.k_b_t()`,
  `thermostat.gamma()`). **Does not need a trait.**
- **Callback-firing-count test**: Verifies the callback is invoked
  exactly once per `step()` and that the RNG advances by the
  expected number of draws. **Does not need temperature or seed
  introspection.**
- **Reproducibility test**: Verifies that two `LangevinThermostat`
  instances constructed with the same seed produce identical
  trajectories. Uses concrete-type construction directly. **Does
  not need a trait.**
- **Multi-component diagnostic helper**: Doesn't exist in Phase 1.
  No use case yet.
- **Generic test helper that takes `T: PassiveComponent + Diagnose`**:
  Doesn't exist in Phase 1. No use case yet.

**Conclusion**: Phase 1 has *zero* generic consumers of
introspection. Every Phase 1 test that needs to read parameters
back can do so via concrete-type accessors on `LangevinThermostat`.

This is the YAGNI question for Decision 4: ship a `Diagnose` trait
that has no current consumer, defer it to when one appears, or
ship a minimal version that doesn't lock in a structure?

#### Scheme A — Defer `Diagnose` entirely to Phase 2+

Don't ship `Diagnose` in Phase 1. The Phase 1 tests use
concrete-type accessors on `LangevinThermostat` directly. When
Phase 2 (or later) introduces a real generic introspection
consumer — for example, a "summarize all components in this
stack" debug helper, or a generic statistical-test runner that
wants to assert temperatures across heterogeneous thermostats —
*that* is when we add the trait, with its shape informed by the
real consumer's needs.

**Pros**:
- **Smallest Phase 1 surface**. Less to maintain, less to test,
  fewer files.
- **No premature design**. The trait shape is informed by an
  actual consumer when the consumer appears, not by a guess
  about what consumers might want.
- **Adding a trait later is non-breaking**. Existing code that
  doesn't implement `Diagnose` keeps working; new code that
  needs the trait adds it incrementally.
- Honors Decision 1's "orthogonal trait *if needed*" framing
  literally — Phase 1 doesn't need it, so Phase 1 doesn't ship
  it.

**Cons**:
- The Decision 1 promise becomes purely architectural ("we have
  a place to put it") with no concrete artifact in Phase 1.
- A user reading the Phase 1 crate sees no "what's in this
  stack?" debug helper at all — they have to print things
  manually with concrete-type accessors.
- Phase 2+ design pressure on the trait shape will be lower
  because the trait will be designed in isolation, not against
  shipping code.

#### Scheme B — Minimal `Diagnose` with one method (`diagnostic_summary -> String`)

Ship the trait in Phase 1 with exactly one required method:

```rust
pub trait Diagnose {
    /// Human-readable one-line summary of this component's
    /// configuration. Used for debug logging, error messages,
    /// and visualization HUDs.
    fn diagnostic_summary(&self) -> String;
}
```

Component implementation:
```rust
impl Diagnose for LangevinThermostat {
    fn diagnostic_summary(&self) -> String {
        format!(
            "LangevinThermostat(kT={:.6}, n_dofs={}, seed={})",
            self.k_b_t,
            self.gamma.len(),
            self.seed,
        )
    }
}
```

User code:
```rust
let thermostat = LangevinThermostat::new(gamma, kT, seed);
println!("{}", thermostat.diagnostic_summary());
// → "LangevinThermostat(kT=1.000000, n_dofs=1, seed=42)"
```

For multi-component setups, the `PassiveStack` could optionally
get a `Display` impl that calls `diagnostic_summary()` on each
component (out of scope for Decision 4, but trivially addable
later).

**Pros**:
- **Tiny commitment** — one trait, one method, ~5 LOC of crate
  code, ~5 LOC per implementing component.
- **Instantly useful** for debug logging, test failure messages,
  and error reporting. A test that fails with
  "LangevinThermostat(kT=1.0, n_dofs=1, seed=42) failed
  equipartition: measured 0.495 vs expected 0.5" is much easier
  to debug than one that just says "failed."
- **Doesn't lock in any specific introspection structure**. The
  string is opaque to programmatic use; future Phase 2+ can add
  typed accessor methods (`fn diagnostic_temperature(&self) ->
  Option<f64>`, etc.) as additive trait methods with default
  `None` implementations. Non-breaking.
- **Establishes the trait as a real, shipping artifact** in
  Phase 1, so future phases extend it rather than birthing it.

**Cons**:
- The `String` return type is opaque — programmatic access to
  individual fields (kT, gamma, seed) still requires concrete-
  type downcasting, which is inelegant for *generic* test
  code.
- Adds one more file/section to the Phase 1 crate that has no
  test consumer of its own — the `diagnostic_summary` method is
  used at debug time, not asserted on. Mild YAGNI tension.

#### Scheme C — Full `Diagnose` with typed Option-returning methods

Ship the full trait sketch from Decision 1:

```rust
pub trait Diagnose {
    fn diagnostic_summary(&self) -> String;
    fn diagnostic_temperature(&self) -> Option<f64> { None }
    fn diagnostic_seed(&self) -> Option<u64> { None }
    fn diagnostic_damping(&self) -> Option<&DVector<f64>> { None }
}
```

Components opt into specific methods by overriding from the
default `None`.

**Pros**:
- Generic test helpers can call `t.diagnostic_temperature()`
  without downcasting.
- The introspection contract is explicit at the trait level.
- Matches Decision 1's literal sketch.

**Cons**:
- **Premature**: Phase 1 has no generic consumers of any of
  these methods. The trait surface is designed for hypothetical
  Phase 2+ users.
- **Locks in specific physical concepts**: `temperature` is a
  thermostat concept. A bistable element from Phase 3 has a
  `barrier_height`, not a `temperature`. A stochastic driver
  from Phase 2 has a `frequency` and `amplitude`, not a
  `temperature`. We'd be adding methods to the trait that don't
  generalize, then adding more methods later for each new
  introspection key, then either bloating the trait or
  fragmenting it.
- **Mild violation of Decision 1's spirit**. Decision 1 said
  the *core* trait stays minimal and physical concepts go on
  *orthogonal* traits. Scheme C makes `Diagnose` itself a
  thermostat-specific trait by baking thermostat methods into
  it. If anything, the right Phase 2+ shape is *multiple*
  orthogonal introspection traits (`HasTemperature`,
  `HasPeriodicSchedule`, `HasBarrierHeight`), not one mega-
  trait.

#### Recommendation: **Scheme B — minimal `Diagnose` with `diagnostic_summary` only**

Confidence: medium-high. It's a YAGNI judgment call where the
right answer depends on how much we trust Phase 1's design
pressure to inform the future. Four reasons for B over A and C:

1. **Tiny cost, real value**. ~10 LOC of crate code total
   (trait + one impl) buys debug-printable components forever.
   Test failure messages get qualitatively better. The
   LOC-to-value ratio is excellent.

2. **Doesn't lock in introspection structure**. The string
   return type is honest — "components describe themselves
   however they want" — and future typed accessors can be
   added as additive trait methods with default
   implementations. Non-breaking, structurally compatible with
   Scheme C as a future evolution.

3. **Avoids Scheme C's category error**. `temperature`,
   `damping`, and `seed` are *thermostat* concepts, not
   *passive component* concepts. Baking them into a generic
   `Diagnose` trait would imply that every passive component
   has a temperature, which is false (drivers, ratchets,
   custom drag laws don't). The "multiple narrow orthogonal
   traits" pattern (`HasTemperature`, etc.) is the right
   eventual shape, but it's not Phase 1 work.

4. **Honors Decision 1 concretely**. Decision 1 said
   introspection has a home if needed; Scheme B ships that
   home with one method, satisfying the promise without
   overdesigning. Scheme A defers the promise; Scheme B
   delivers a minimal version of it.

**Why I'm not recommending A**: it's the most defensible YAGNI
position but it leaves the test failure messages of Phase 1
worse than they need to be, and it pushes the design of the
trait to a moment when the design pressure may be diffuse.
Scheme B's 10-LOC cost is small enough that the YAGNI argument
doesn't dominate.

**Why I'm not recommending C**: it locks in thermostat-specific
methods in a trait that's supposed to be orthogonal to the
thermostat-vs-non-thermostat distinction. Scheme C is the right
*direction* (typed introspection eventually exists) but the
*shape* needs to be multiple narrow traits, not one mega-trait,
and that design conversation belongs to Phase 2+ when there are
real heterogeneous components to inform it.

#### Sub-decisions inside Decision 4

- **Trait name**: `Diagnose`. Locked from Decision 1, not
  relitigated.
- **Method name**: `diagnostic_summary`. Considered: `summary`
  (too generic), `describe` (too verbose), `to_string` (clashes
  with `Display`), `display_summary` (verbose). `diagnostic_*`
  prefix matches the typed-accessor convention from Decision
  1's sketch (`diagnostic_temperature`, etc.) so future phases
  extend cleanly.
- **Return type**: `String`, owned. Considered: `Cow<'static,
  str>` (fancy), `&'static str` (impossible — formatting needs
  allocation), `impl Display` (clean but harder to use in
  format strings). Owned `String` is the simplest and most
  compatible with downstream consumers (println!, format!,
  thiserror).
- **Default implementation**: none — `diagnostic_summary` is
  a required method. Components that don't implement it just
  don't implement `Diagnose`. Considered: providing a default
  that returns `format!("{}", std::any::type_name::<Self>())`
  — rejected because the type name alone is not useful, and a
  forced `unimplemented!()`-by-default would be a footgun.
- **Trait location**: `sim_thermostat::Diagnose`, declared in
  `lib.rs` for Phase 1 (one trait, no submodule needed). When
  Phase 2+ adds more orthogonal introspection traits, they can
  share a `diagnose` submodule.
- **Component opt-in**: every Phase 1 component that ships
  with the crate (just `LangevinThermostat`) implements
  `Diagnose`. User-defined components can implement it or not
  at their discretion.
- **`PassiveStack` does not implement `Diagnose` in Phase 1**.
  Reason: the stack's diagnostic summary depends on its
  contents, and we have no consumer asking for a stack-level
  summary yet. Trivially addable later as
  `impl Display for PassiveStack` that walks the components.
  YAGNI.
- **Send + Sync bounds on the trait**: not added. Components
  that implement `PassiveComponent` are already `Send + Sync +
  'static`, so any `T: PassiveComponent + Diagnose` is also
  Send + Sync. Adding the bounds to `Diagnose` itself would be
  redundant noise.

#### DECISION (user confirmed): **Scheme B — minimal `Diagnose` with `diagnostic_summary` only**

User cited the 80-20 framing: bulk of the value (debug-printable
components, qualitatively better test failure messages) for a
fraction of the cost (~10 LOC). Confirmed.

Final API surface:

```rust
pub trait Diagnose {
    /// Human-readable one-line summary of this component's
    /// configuration. Used for debug logging, error messages,
    /// and visualization HUDs.
    fn diagnostic_summary(&self) -> String;
}

impl Diagnose for LangevinThermostat {
    fn diagnostic_summary(&self) -> String {
        format!(
            "LangevinThermostat(kT={:.6}, n_dofs={}, seed={})",
            self.k_b_t,
            self.gamma.len(),
            self.seed,
        )
    }
}
```

Future Phase 2+ extensions are additive — typed accessor
methods can be added with default `None` implementations
without breaking existing code.

#### Status

- **Decision 4 RESOLVED.** Scheme B confirmed. The `Diagnose`
  trait ships in Phase 1 with one method, every Phase 1
  component implements it, and the door is open for additive
  Phase 2+ extension into multiple narrow orthogonal traits
  (`HasTemperature`, etc.) when real heterogeneous components
  inform the shape.
- No code, no Cargo.toml changes, no new files written beyond
  this document.

---

### Decision 5 (2026-04-09): Public test utilities

- **Question**: What statistical-test assertion helpers does the
  chassis ship as `pub` test utilities? The recon round
  established (item 7 part 9) that the thermostat's
  equipartition test will be the workspace's *first*
  statistical-sampling validation test — no convention to
  inherit, so the chassis must establish one. The helpers must
  be reusable across phases (Phase 2 free + articulated body
  equipartition, Phase 3 Kramers escape rate, Phase 4 Ising
  joint distribution, Phase 5 EBM target match), not specific
  to Phase 1's 1-DOF damped harmonic oscillator.
- **Why this is decision 5**: Phase 1's equipartition test is
  the test infrastructure consumer. Until Decision 5 settles
  what helpers exist, the Phase 1 spec cannot describe how the
  test code is structured. Decision 5 also surfaces a
  Phase-1-validation finding (see "Side finding" below) that
  affects the master plan's validation parameters and will
  shape the spec.

#### Side finding (surfaced during Decision 5 reasoning) — the 10⁵ step count may be insufficient

**This belongs in the Phase 1 spec, not the chassis design,
but I want to flag it here because it came up while thinking
about what test helpers we need.** The master plan §The Gap
Phase 1 validation parameters are: M=1, k_spring=1, γ=0.1,
k_B·T=1, h=0.001, 10⁵ steps after burn-in. The recon log
part 2 claimed "discretization temperature error ≈ 10⁻⁴ —
well below the ~10⁻² sampling-error tolerance for 10⁵
samples," and called the test "should pass with margin."

The "10⁻² tolerance for 10⁵ samples" calculation assumes
**independent samples**, which they are not. For a 1D damped
harmonic oscillator, the velocity-squared autocorrelation
decays at rate `2γ/M`, giving an integrated autocorrelation
time of approximately `M / (2γh)` steps. With M=1, γ=0.1,
h=0.001:

```
τ_int ≈ 1 / (2 · 0.1 · 0.001) = 5000 steps
```

So 10⁵ steps gives an *effective* sample count of:

```
N_eff ≈ 10⁵ / (1 + 2·5000) ≈ 10
```

For chi-squared distributed `½ M v²` (mean `½ kT`, standard
deviation `(½ kT)·√2`), the standard error of the sample mean
over N_eff = 10 effective samples is
`(½ kT)·√2/√10 ≈ 0.45 · (½ kT)` — about **±45% relative to
the expected mean ½kT**, not ±2%.

**Tolerance convention** (locked in by doc review M1,
2026-04-09): all sampling-error tolerances in the thermo line
are expressed as a fraction of the expected mean (here `½kT`),
not as a fraction of `kT`. The pre-correction wording mixed
the two and was off by a factor of 2.

**Implication**: the part-2 reasoning was *too optimistic*. The
test as currently parameterized cannot pass with margin at ±2%
of `½kT` — the statistical noise (±45% of `½kT`) dominates the
discretization bias by more than an order of magnitude. Three
plausible fixes (Phase 1 spec design question, not Decision 5):

- **(α)** Increase total step count by ~100× to ~10⁷.
- **(β)** Use multiple independent trajectories: e.g., 100 runs
  of 10⁵ steps each, with different seeds. Each run gives one
  independent estimate; the standard error of the 100-estimate
  mean is `run_std / √100`, bringing the combined std error to
  about **±4.5% of `½kT`** — in the right neighborhood for a
  meaningful gate.
- **(γ)** Loosen the tolerance to match reality (~5-10% of
  `½kT`) and accept that Phase 1 cannot detect discretization
  bias.

I lean toward (β) — multiple independent trajectories — because
it (i) avoids the autocorrelation analysis entirely, (ii) gives
trivially clean statistics, (iii) exercises the seed/RNG path
under normal use, and (iv) total step count remains 10⁷ which
runs in seconds. **But this is a Phase 1 spec decision, not a
Decision 5 chassis decision**, so I'm flagging it here and
deferring the actual choice to spec time.

The reason it surfaces in Decision 5: the choice of fix
determines whether the test helpers need autocorrelation
analysis (α and γ require it; β doesn't). My recommendation
below assumes β-style "multiple independent trajectories" is
the likely fix, which keeps the Phase 1 helper surface
minimal. If the spec ends up choosing α instead, we add
`integrated_autocorrelation_time` as an additive helper later
— non-breaking.

#### What helpers does the chassis actually need to ship?

Walking through the candidate helpers:

- **`assert_within_n_sigma(measured, expected, std_error,
  n_sigma, description)`** — the assertion + diagnostic
  message. Takes the standard error pre-computed by the
  caller. Prints everything needed to debug a failure.
  **Required for Phase 1.**
- **`WelfordOnline`** — streaming mean + variance accumulator.
  For tests that run 10⁵–10⁷ steps and don't want to allocate
  a giant `Vec<f64>` just to compute mean and variance at the
  end. **Required for Phase 1** (the alternative is allocating
  a Vec or rolling `(sum, sum_sq)` manually with the
  numerical-stability footgun that Welford's algorithm exists
  to avoid).
- **`sample_stats(&[f64]) -> (mean, variance)`** —
  convenience wrapper around `WelfordOnline` for the case
  where the user already has a Vec. **Trivially useful.**
- **`integrated_autocorrelation_time(&[f64]) -> f64`** —
  Sokal-style automatic-windowing autocorrelation analysis.
  **DEFERRED to Phase 2+** — only needed if the Phase 1 spec
  chooses option (α) or (γ) above. If it chooses (β) (multiple
  independent trajectories), this is unnecessary.
- **`burn_in_skip(&[f64], n) -> &[f64]`** — discard the first
  N samples. **Not shipping** — it's literal slice indexing
  (`&data[n..]`) and a helper would just be noise.
- **Domain-specific helpers like `assert_equipartition(...)`**
  — explicitly **not shipping**. Decision 1 (broad trait,
  no physics in the chassis) and Decision 4 (no thermostat-
  specific concepts in `Diagnose`) both reject baking
  physics into the chassis. The same logic applies to test
  helpers: ship statistical primitives, let the test code
  apply them to specific physics.

#### Three schemes

**Scheme A — minimal: just `assert_within_n_sigma`**

Ship one assertion helper. The test author computes mean,
variance, and standard error themselves with manual loops or
inline math.

**Pros**: smallest surface, ~10 LOC of crate code.

**Cons**: every Phase 1+ statistical test re-implements
mean/variance computation. Welford's algorithm is non-trivial
to write correctly from scratch — the naive `(sum, sum_sq)`
approach has well-known catastrophic-cancellation failure
modes for series with large mean and small variance, which is
*exactly* the equipartition case (mean ≈ 0.5, variance also
~0.5 but visible only after subtracting mean²). Asking every
test author to re-implement this correctly is asking for
silent statistical bugs.

**Scheme B — focused toolkit: assertion + WelfordOnline + sample_stats**

Ship the three helpers above:
```rust
pub fn assert_within_n_sigma(
    measured: f64,
    expected: f64,
    standard_error: f64,
    n_sigma: f64,
    description: &str,
);

pub struct WelfordOnline { /* ... */ }
impl WelfordOnline {
    pub fn new() -> Self;
    pub fn push(&mut self, x: f64);
    pub fn count(&self) -> usize;
    pub fn mean(&self) -> f64;
    pub fn variance(&self) -> f64;  // unbiased (n-1 denominator)
    pub fn std_error_of_mean(&self) -> f64;
}

pub fn sample_stats(data: &[f64]) -> (f64, f64);
```

The streaming `WelfordOnline` is the primary tool for tests
that don't want to materialize all samples; `sample_stats` is
the convenience for tests that already have a slice.

**Pros**: covers the 80-20 of statistical test needs; correct
implementations of mean/variance that Phase 1+ tests don't
have to re-derive; numerical-stability built in (Welford);
standard error of mean is a one-line accessor.

**Cons**: ~80 LOC of crate code (Welford has more state than
the naive version). One more module to maintain.

**Scheme C — domain-specific: helpers per physics**

```rust
pub fn assert_equipartition(
    samples: &[f64],
    mass: f64,
    kT: f64,
    n_sigma: f64,
);
pub fn assert_kramers_rate(...);
pub fn assert_ising_distribution(...);
```

**Rejected** for the same reason Decisions 1 and 4 reject
baking physics into traits: it couples the chassis to specific
test scenarios, fragments as new tests appear, and forces
chassis edits for every new physics validation. The chassis
should ship statistical primitives; physics goes in the test
code itself.

#### Recommendation: **Scheme B — focused toolkit**

Confidence: high. Five reasons:

1. **The numerical stability case is real**. Naive
   `(sum, sum_sq)` mean/variance computation fails
   catastrophically for the equipartition case (large mean,
   variance comparable to mean²). Welford's algorithm exists
   precisely to avoid this. Asking Phase 1+ test authors to
   reimplement Welford correctly is the kind of silent-bug
   trap sharpen-the-axe forbids.

2. **The streaming use case is real**. 10⁵ steps × 8 bytes
   = 800 KB per quantity tracked. Multiply by 10 quantities
   for parameter sweeps = 8 MB per test. With 100
   independent trajectories per parameter combination (per
   the Phase 1 spec sketch above) = 800 MB. Materializing
   everything is wasteful when streaming computes the same
   answer in O(1) memory per quantity.

3. **Ship-once amortization**. Phase 2 (free + articulated
   body equipartition), Phase 3 (Kramers rate), Phase 4
   (Ising joint distribution), Phase 5 (EBM target match)
   all need exactly these three helpers. Every additional
   helper added now amortizes across 4+ future phases.

4. **Established test pattern**. Once `WelfordOnline` +
   `assert_within_n_sigma` is the convention, every later
   thermo-line statistical test reads structurally
   identically: build a Welford accumulator per quantity,
   push samples in the inner loop, assert at the end. This
   is the "mechanical" property the user explicitly asked
   for at the test level.

5. **No physics baked in**. The helpers operate on `f64` and
   `&[f64]`; the physics interpretation lives in the test
   code. This honors Decision 1 + Decision 4's "chassis
   stays minimal, physics goes in components/tests" line.

#### Sub-decisions inside Decision 5

- **Module path**: `sim_thermostat::test_utils`. `pub mod
  test_utils;` declared in `lib.rs`. Public, not feature-
  gated. Matches the workspace convention (sim-ml-bridge,
  sim-core do not feature-gate test utilities).
- **`WelfordOnline` field privacy**: fields are private
  (`count`, `mean`, `m2`); only the public accessor methods
  (`mean()`, `variance()`, `std_error_of_mean()`,
  `count()`) are exposed. Reason: the M2 field is an
  intermediate quantity, not a meaningful statistic to
  expose.
- **Variance is unbiased (n-1 denominator)**. The biased
  (n) version is rarely what statistical tests want;
  unbiased is the default for sample-based estimates.
- **Standard error of mean accessor**: `std_error_of_mean()
  -> f64`. Returns `(variance / count).sqrt()`. Convenience
  — saves the test author from typing
  `(welford.variance() / welford.count() as
  f64).sqrt()` at every site.
- **`sample_stats` returns `(mean, variance)`** in that
  order. Considered: `(mean, std)`, `(mean, std_error)`,
  named struct. Tuple is the simplest; (mean, variance) is
  the most general (the caller can take sqrt or divide by N
  themselves).
- **`assert_within_n_sigma` description parameter**: takes
  `&str` (not `String`), borrows for the assertion message.
  Test authors pass string literals; no allocation in the
  pass case.
- **Failure message format**: includes the description, the
  measured value, the expected value, the standard error,
  the computed |z| score, and the n_sigma threshold. All
  numerics in scientific notation with 6 significant digits
  for precision in the failure message.
- **Welford's `push` takes `f64` by value, not `&f64`**.
  f64 is Copy, no reason to take a reference.
- **`reset()` ships in Phase 1** (added by doc review M4,
  2026-04-09). Without it, the streaming-Welford burn-in
  story is broken: the chassis explicitly does not ship a
  `burn_in_skip` helper because "it's literal slice
  indexing," but slice indexing is unavailable when the
  samples were never materialized in the first place. The
  natural pattern is *push during burn-in → reset → push
  the real samples → assert*, which requires `reset()`.
  One-line implementation (`*self = Self::new();`), real
  ergonomic win for the streaming use case the chassis was
  designed around.
- **`merge(&other)` ships in Phase 1** (added by doc review
  M4, 2026-04-09). Combines two independent `WelfordOnline`
  accumulators into one global statistic using the
  Chan/Pébay parallel-merge formula (Chan, Golub, LeVeque
  1979; Pébay 2008 generalization). Required by option (β)
  of the Phase 1 validation parameter fix — 100 independent
  trajectories, each with its own per-trajectory accumulator,
  combined into a single 100-trajectory mean + variance —
  and by every Phase 4+ parallel-env test that wants to
  fold per-env statistics together. Without `merge`, option
  (β) has to materialize per-trajectory means in a `Vec<f64>`
  and re-Welford them, which loses the streaming property
  for the variance estimate. ~6 LOC implementation:
  ```rust
  pub fn merge(&mut self, other: &Self) {
      if other.count == 0 { return; }
      if self.count == 0 { *self = other.clone(); return; }
      let n_a = self.count as f64;
      let n_b = other.count as f64;
      let n   = n_a + n_b;
      let delta = other.mean - self.mean;
      self.mean += delta * (n_b / n);
      self.m2   += other.m2 + delta * delta * (n_a * n_b / n);
      self.count += other.count;
  }
  ```
  Requires `WelfordOnline: Clone`, which is trivial (three
  `Copy` fields). Tested against the equivalent
  push-everything-into-one-accumulator path for byte-exact
  agreement.
- **`integrated_autocorrelation_time` is NOT in Phase 1**.
  Deferred to whichever later phase needs single-trajectory
  correlated-sample tests. If Phase 1 spec ends up choosing
  fix (α) (10× longer trajectories) over fix (β) (multiple
  independent trajectories), we add this as an additive
  helper at that point. Non-breaking.
- **No domain-specific helpers** (`assert_equipartition`,
  etc.). Test code applies the primitives to specific
  physics; the primitives stay general.

#### Open follow-on triggered by this decision

- **Phase 1 spec must choose between (α), (β), and (γ)** for
  the validation parameter set. My weak recommendation is
  (β) — multiple independent trajectories — because it
  avoids autocorrelation analysis, exercises the seed/RNG
  path under normal use, and gives clean statistics. But
  this is a spec decision, not a chassis decision. Decision
  5 ships helpers compatible with all three options.

- **The master plan §The Gap Phase 1 validation parameters
  paragraph needs revision** to reflect the autocorrelation
  finding. I will not edit the master plan as part of
  Decision 5 — that's a separate documentation update we
  can do at the end of the chassis design round, alongside
  any other recon/master-plan updates.

#### DECISION (user confirmed): **Scheme B — focused toolkit (ship Welford's algorithm in the chassis)**

`WelfordOnline` is the standard numerically-stable streaming
variance accumulator (Welford 1962, used internally by numpy
and scipy). Shipped once in the chassis so Phase 1+ test
authors never have to reimplement it correctly. Final API
surface:

```rust
// In sim_thermostat::test_utils

pub fn assert_within_n_sigma(
    measured: f64,
    expected: f64,
    standard_error: f64,
    n_sigma: f64,
    description: &str,
);

#[derive(Clone)]
pub struct WelfordOnline { /* count, mean, m2 — all private */ }
impl WelfordOnline {
    pub fn new() -> Self;
    pub fn push(&mut self, x: f64);
    pub fn reset(&mut self);                // discard all samples (burn-in)
    pub fn merge(&mut self, other: &Self);  // Chan/Pébay parallel merge
    pub fn count(&self) -> usize;
    pub fn mean(&self) -> f64;
    pub fn variance(&self) -> f64;          // unbiased (n-1)
    pub fn std_error_of_mean(&self) -> f64; // sqrt(variance / count)
}

pub fn sample_stats(data: &[f64]) -> (f64, f64); // (mean, variance)
```

Ships in Phase 1; covers the 80-20 of statistical-test needs;
amortizes across Phase 2-5 statistical tests.

#### Status

- **Decision 5 RESOLVED.** Scheme B confirmed. Welford's
  algorithm + assertion helper + sample_stats convenience are
  the chassis test utilities for Phase 1.
- **Side finding deferred to documentation pass**: the master
  plan's 10⁵-step validation parameter calculation needs
  revision (autocorrelation makes effective N ≈ 10, not 10⁵).
  Will be addressed at the end of the chassis design round
  alongside any other documentation updates.
- No code, no Cargo.toml changes, no new files written
  beyond this document.

---

### Decision 6 (2026-04-09): Crate layout

- **Question**: What is the on-disk layout of `sim/L0/thermostat/`
  at Phase 1? Specifically: how many files in `src/`, what
  goes in each, where do unit tests vs integration tests live,
  what's in the crate-level rustdoc on `lib.rs`?
- **Why this is decision 6**: Decisions 1-5 settled the
  *contents* of the chassis (trait, composer, clone handling,
  introspection, test utilities). Decision 6 settles the
  *organization*. Until this is resolved, the Phase 1 spec
  cannot reference file paths.
- **Constraint from item 8**: Crate location is
  `sim/L0/thermostat/`, package name `sim-thermostat`. Deps
  are `sim-core`, `nalgebra`, `rand`, `rand_chacha`,
  `rand_distr`. This is locked.
- **What lives in the crate** (sum of Decisions 1-5):
  - `PassiveComponent` trait (Decision 1)
  - `PassiveStack` + `PassiveStackBuilder` + `install` +
    `install_per_env` (Decisions 2 + 3)
  - `Diagnose` trait (Decision 4)
  - `LangevinThermostat` struct + impl (the Phase 1 component)
  - `WelfordOnline` + `assert_within_n_sigma` + `sample_stats`
    test utilities (Decision 5)

#### Reference architecture: ml-bridge

`sim-ml-bridge` has the following top-level layout
(from earlier recon):

```
sim/L0/ml-bridge/
├── Cargo.toml
├── src/
│   ├── lib.rs                # door
│   ├── env.rs                # Environment trait
│   ├── space.rs              # Action/Observation spaces
│   ├── replay_buffer.rs
│   ├── autograd_policy.rs
│   ├── cem.rs                # CEM algorithm
│   ├── sac.rs                # SAC algorithm
│   ├── ppo.rs                # PPO algorithm
│   ├── td3.rs                # TD3 algorithm
│   └── reinforce.rs          # REINFORCE algorithm
└── tests/
    └── competition.rs        # cross-algorithm integration test
```

The pattern: **flat top-level files in `src/`, one concept per
file**. Each algorithm is its own file. Traits and infrastructure
(env, space) are also top-level files. `lib.rs` is a door — it
contains crate docs and re-exports, no type definitions.
Integration tests live in `tests/`.

The thermo crate is a structural sibling to ml-bridge, so its
layout should mirror this pattern unless there's a strong reason
to deviate.

#### Three schemes

**Scheme A — flat ml-bridge style**

```
sim/L0/thermostat/
├── Cargo.toml
├── src/
│   ├── lib.rs           # door: crate docs + re-exports only
│   ├── component.rs     # PassiveComponent trait
│   ├── stack.rs         # PassiveStack + PassiveStackBuilder
│   ├── diagnose.rs      # Diagnose trait
│   ├── langevin.rs      # LangevinThermostat
│   └── test_utils.rs    # WelfordOnline + assertion helpers
└── tests/
    └── langevin_thermostat.rs   # Phase 1 integration tests
```

`lib.rs` looks like:
```rust
//! Stochastic and passive force chassis for sim-core.
//!
//! [crate-level docs — see "Sub-decisions" for what goes here]

mod component;
mod stack;
mod diagnose;
mod langevin;
pub mod test_utils;

pub use component::PassiveComponent;
pub use stack::{PassiveStack, PassiveStackBuilder};
pub use diagnose::Diagnose;
pub use langevin::LangevinThermostat;
```

**Pros**:
- **Matches ml-bridge structurally** — the user explicitly
  named ml-bridge as the reference architecture for this
  crate. Engineers reading the workspace see two L0 sibling
  crates with identical layouts.
- **Architecture visible from `ls`** — a directory listing
  reveals the chassis (component/stack/diagnose), the Phase
  1 component (langevin), and the test infrastructure
  (test_utils) in five lines.
- **Each file has one concern** — easy to review, easy to
  diff, easy to navigate.
- **Trivial to extend** — Phase 2+ adds new component files
  (`baoab.rs`, `brownian_ratchet.rs`) at the top of `src/`,
  same as ml-bridge adds new algorithms.
- `lib.rs` stays small (under 50 LOC including docs and
  re-exports) — easy to read at a glance.

**Cons**:
- 5 source files for ~400 LOC of code feels like a lot of
  files. (It isn't — ml-bridge has 10+ files for less code
  per file.)

**Scheme B — chassis/components hierarchical**

```
sim/L0/thermostat/
├── Cargo.toml
├── src/
│   ├── lib.rs
│   ├── chassis/
│   │   ├── mod.rs
│   │   ├── component.rs    # PassiveComponent trait
│   │   ├── stack.rs        # PassiveStack
│   │   └── diagnose.rs     # Diagnose trait
│   ├── components/
│   │   └── langevin.rs     # LangevinThermostat
│   └── test_utils.rs
└── tests/
    └── langevin_thermostat.rs
```

**Pros**:
- Visually separates chassis (the swappable bolt patterns)
  from components (the swappable implementations).
- Phase 2+ adds files only inside `components/`, leaving
  `chassis/` untouched — emphasizes the R34 stability
  property.

**Cons**:
- **Deeper directory structure than the existing sim/L0
  crates use**. None of `sim-types`, `sim-simd`, `sim-core`,
  `sim-mjcf`, `sim-urdf`, `sim-ml-bridge` use a
  chassis/components split. We'd be inventing a new
  organizational pattern for this one crate.
- **Module path noise**: `sim_thermostat::components::langevin::LangevinThermostat`
  vs. `sim_thermostat::LangevinThermostat`. The lib.rs
  re-exports flatten this for users, but internal code paths
  get longer.
- **Premature** for Phase 1 with one component. The split
  pays off only when there are many components and a clear
  chassis-vs-component review boundary. Phase 1 has neither.

**Scheme C — single lib.rs**

```
sim/L0/thermostat/
├── Cargo.toml
├── src/
│   └── lib.rs   # everything: trait, stack, diagnose, langevin, test_utils
└── tests/
    └── langevin_thermostat.rs
```

**Pros**:
- Smallest possible file count.

**Cons**:
- **Diverges from ml-bridge** with no offsetting benefit.
- **Architecture not visible from `ls`** — a directory
  listing shows only `lib.rs`, hiding the chassis-component
  separation we've been designing for the entire chassis
  document.
- **lib.rs grows linearly with components**. Phase 2 doubles
  it; Phase 4-5 might triple it. Hard to navigate as a
  ~1000+ LOC file.
- **No file-level review boundary**. Reviewing a change to
  `LangevinThermostat` means reading a diff that touches
  the same file as the trait, the stack, and the test
  utilities — they're not visually separated.

#### Recommendation: **Scheme A — flat ml-bridge style**

Confidence: high. Three reasons:

1. **ml-bridge precedent is unambiguous and the user named
   it explicitly as the reference architecture**. Diverging
   from it would force engineers to learn two layouts for two
   structurally-parallel L0 crates. The "readability and
   organization are the highest priority" principle is
   directly served by matching the existing pattern.

2. **The "five files reveal the architecture" property is
   real and matters**. A first-time reader who runs `ls
   sim/L0/thermostat/src/` sees `component.rs`, `stack.rs`,
   `diagnose.rs`, `langevin.rs`, `test_utils.rs` — and they
   immediately know the chassis is in the first three files,
   the implementation is in the fourth, and the test
   helpers are in the fifth. This is the
   architecture-from-types property the user has asked for
   repeatedly.

3. **Mechanical extension to Phase 2+**. Adding `baoab.rs`,
   `brownian_ratchet.rs`, `stochastic_driver.rs` is
   one-file-per-component, no module restructuring. Matches
   how ml-bridge added cem/sac/ppo/td3/reinforce
   incrementally.

**Why I'm not recommending B**: it invents a hierarchy that
no other sim/L0 crate uses, and the "chassis vs components"
visual separation is already achieved in Scheme A by file
naming (the chassis files have generic names —
`component.rs`, `stack.rs`, `diagnose.rs` — while the
component files have physics-specific names —
`langevin.rs`). The hierarchy adds module path noise without
adding clarity.

**Why I'm not recommending C**: it's the simplest layout but
the "everything in one file" property defeats the
chassis-component-test_utils separation we've been
designing. The cost of 4 extra files is trivial; the loss
of file-level review boundaries is not.

#### Sub-decisions inside Decision 6

- **`lib.rs` is a door, not a workshop**. Only crate-level
  rustdoc and `pub use` re-exports. No type definitions, no
  function definitions, no impl blocks. Matches ml-bridge.
- **lib.rs rustdoc contents**:
  1. Crate purpose paragraph (1-2 lines)
  2. Architecture summary (~10 lines): name the chassis
     (`PassiveComponent` + `PassiveStack` + `Diagnose`), name
     the Phase 1 component (`LangevinThermostat`), name the
     test infrastructure (`test_utils`).
  3. Quick-start example: a short code block showing
     `PassiveStack::builder().with(LangevinThermostat::new(...)).build().install(&mut model)`.
  4. Link to `docs/thermo_computing/THERMO_CHASSIS_DESIGN.md`
     for the design rationale.
  5. Link to `docs/thermo_computing/PHASE_1_LANGEVIN_THERMOSTAT_SPEC.md`
     once that exists.
- **Module visibility**: `mod component;`, `mod stack;`,
  `mod diagnose;`, `mod langevin;` are all private — the
  modules themselves aren't part of the public API, only the
  re-exports are. **Exception**: `pub mod test_utils;` is
  public because users of the crate can import the test
  helpers directly (`use sim_thermostat::test_utils::WelfordOnline;`).
- **Re-exports**: at minimum,
  `pub use component::PassiveComponent;`,
  `pub use stack::{PassiveStack, PassiveStackBuilder};`,
  `pub use diagnose::Diagnose;`,
  `pub use langevin::LangevinThermostat;`. Five public types
  total at Phase 1; matches the surface area of a focused
  Rust crate.
- **Unit tests live inside each module file** with
  `#[cfg(test)] mod tests { ... }`. Standard Rust convention.
  Matches ml-bridge.
- **Integration tests live in `sim/L0/thermostat/tests/`**,
  not in `sim-conformance-tests`. Reason: the thermo tests
  depend on `rand`, `rand_chacha`, `rand_distr`, which
  `sim-conformance-tests` does not pull in (verified in
  item 7 part 9). Adding these deps to
  `sim-conformance-tests` to host one crate's tests would
  pollute the test crate's deps for every other sim test.
  ml-bridge precedent: it has its own
  `sim/L0/ml-bridge/tests/competition.rs` separate from
  `sim-conformance-tests`. The thermo crate follows the
  same pattern.
- **Phase 1 integration test file**: single file
  `tests/langevin_thermostat.rs` containing the equipartition
  test, callback-firing-count test, and reproducibility test.
  Split into per-test-category files only when the file grows
  beyond ~500 LOC. For Phase 1 with ~10-20 tests, one file
  is fine.
- **No `examples/` directory in Phase 1**. Examples are a
  Phase 2+ concern (after Phase 1 has shipped a working
  thermostat, an example demonstrating "watch a particle
  thermalize" is valuable — but not before).
- **No `benches/` directory in Phase 1**. Benchmarking is
  not the Phase 1 validation gate; equipartition is. Add
  benches when there's a performance question to answer
  (probably Phase 4+ when coupled bistable arrays start
  stressing the per-step cost).
- **No `README.md` in the crate at Phase 1**. The
  crate-level rustdoc serves the same purpose, and CLAUDE.md
  + the user's "museum-plaque READMEs" preference suggests
  READMEs go in examples, not in core sim crates. (Verified:
  `sim/L0/core`, `sim/L0/ml-bridge`, etc. don't have
  READMEs.) Skip.

#### Final Phase 1 file inventory

```
sim/L0/thermostat/                       (NEW directory)
├── Cargo.toml                           (~20 LOC)
└── src/
    ├── lib.rs                           (~50 LOC: docs + re-exports)
    ├── component.rs                     (~70 LOC: PassiveComponent + Stochastic + tests)
    ├── stack.rs                         (~170 LOC: builder + install + install_per_env + EnvBatch + stochastic gating + tests)
    ├── diagnose.rs                      (~20 LOC: trait + tests)
    ├── langevin.rs                      (~150 LOC: struct + impl + tests)
    └── test_utils.rs                    (~170 LOC: Welford + reset + merge + assertions + tests)
└── tests/
    └── langevin_thermostat.rs           (~250 LOC: integration tests)
```

Total Phase 1 footprint: **~900 LOC** across **8 files**.
Comparable to a small ml-bridge algorithm file plus its
tests. Manageable, mechanical, swappable.

#### DECISION (user confirmed): **Scheme A — flat ml-bridge style**

The thermo crate matches `sim-ml-bridge` structurally: top-level
files in `src/`, one concept per file, `lib.rs` is a door,
integration tests in the crate's own `tests/` directory.
Architecture is observable from `ls sim/L0/thermostat/src/`.

#### Status

- **Decision 6 RESOLVED.** Scheme A confirmed. The Phase 1
  file inventory above is the on-disk layout the crate ships
  with.
- **Updated by doc review M2 (2026-04-09)**: `component.rs`
  grows from ~30 to ~70 LOC (adds the `Stochastic` trait
  and the `as_stochastic` introspection hook on
  `PassiveComponent`). `stack.rs` grows from ~120 to ~160 LOC
  (adds `set_all_stochastic` and the `StochasticGuard` RAII
  type).
- **Updated by doc review S3 (2026-04-09)**: `stack.rs` grows
  from ~160 to ~170 LOC (adds the `EnvBatch` struct and
  `install_per_env` returns it instead of `Vec<Model>`). Total
  Phase 1 footprint moves from ~810 to ~900 LOC.
- No code, no Cargo.toml changes, no new files written beyond
  this document.

---

### Decision 7 (2026-04-09, doc review M2): Stochastic gating for FD/autograd contexts

- **Question**: When the autograd engine perturbs `qpos[i]` and
  re-runs `forward()` to compute `∂qacc/∂qpos[i]` via finite
  differences, the call chain `forward_acc()` → `mj_fwd_passive()`
  → `cb_passive` fires the `PassiveStack` closure, which fires
  every component's `apply()`, which (for stochastic components
  like `LangevinThermostat`) consumes RNG and produces a *fresh*
  noise sample on every call. The baseline and perturbed runs
  therefore see *different* noise samples, and the FD estimate
  `(qacc(qpos+ε) - qacc(qpos)) / ε` is dominated by
  `Var(noise)/ε`, blowing up as `ε → 0`. **What does the chassis
  provide so Phase 5 (differentiable cf-design) and the D3
  headline experiment can recover meaningful derivatives?**
- **Why this is Decision 7**: Decisions 1-6 settled the trait,
  composer, lifecycle, introspection, test utilities, and crate
  layout. The "Phase 5+ caveat" in master plan §The Gap and
  recon log part 4 flagged this issue but punted it. Doc review
  M2 (2026-04-09) escalated it to a chassis-level decision: the
  chassis exists *exactly* so Phase 5 doesn't get caught
  off-guard, and "we'll deal with it later" is the failure mode
  the chassis is designed to prevent. Retrofitting a gating
  mechanism into the trait at Phase 5 would be breaking-change
  territory; doing it now is one trait method + one introspection
  hook + one stack method, with zero implementations to migrate.
- **Constraint from item 8**: Zero sim-core changes. Cannot add
  a "disable cb_passive" flag to `Model`. The gating must live
  entirely inside `sim-thermostat`.
- **Constraint from M5**: Components have no mutable access to
  `Data`. Whatever gating mechanism we add, the component must
  honor it from inside `apply(&self, &Model, &Data, &mut DVector<f64>)`.
- **Observation that drives the recommendation**: Every stochastic
  component on the Research Directions roadmap has
  **state-independent noise**. The Langevin thermostat draws from
  `N(0, 2γkT/h)` — depends on `γ`, `kT`, `h`, none of which
  depend on `qpos` or `qvel`. The flashing-ratchet Brownian motor
  (D1) gates a fixed potential on/off with thermal noise from a
  fixed-`T` bath. Stochastic resonance (D2) injects sub-threshold
  periodic forcing plus thermal bath at fixed `T`. None of these
  has noise that depends on the perturbation variables.

  For state-independent noise, the FD derivative satisfies
  `∂(F_det + ξ)/∂qpos = ∂F_det/∂qpos + 0 = ∂F_det/∂qpos`. So
  **disabling the noise during FD perturbation runs gives the
  exactly-correct derivative for the deterministic part** —
  which, for state-independent noise, is the *complete* answer.

#### Scheme A — Orthogonal `Stochastic` opt-in trait + RAII gating

A small extension trait that stochastic components opt into,
plus a default-`None` introspection hook on `PassiveComponent`
so the stack can find them, plus an RAII guard on `PassiveStack`
that disables-on-construction and re-enables-on-drop.

```rust
// component.rs

pub trait PassiveComponent: Send + Sync + 'static {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>);

    /// If this component has a stochastic part, return a handle to
    /// toggle it. Default: no stochastic part, returns `None`.
    /// Components that implement `Stochastic` override this to return
    /// `Some(self)`.
    fn as_stochastic(&self) -> Option<&dyn Stochastic> { None }
}

pub trait Stochastic: Send + Sync {
    /// When false, `apply()` must produce ONLY this component's
    /// deterministic contribution (e.g., a Langevin thermostat
    /// outputs only `-γ·v`, no noise). When true (the default),
    /// `apply()` produces the full stochastic contribution.
    ///
    /// Used by FD/autograd contexts where the perturbed runs must
    /// be deterministic to recover meaningful derivatives.
    fn set_stochastic_active(&self, active: bool);
    fn is_stochastic_active(&self) -> bool;
}
```

`LangevinThermostat` implements both:

```rust
// langevin.rs

pub struct LangevinThermostat {
    gamma: DVector<f64>,
    k_b_t: f64,
    rng: Mutex<ChaCha8Rng>,
    stochastic_active: AtomicBool,  // M2: defaults to true
}

impl PassiveComponent for LangevinThermostat {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>) {
        let h = model.timestep;
        let active = self.stochastic_active.load(Ordering::Acquire);
        let mut rng = self.rng.lock().unwrap();
        for i in 0..self.gamma.len() {
            let g = self.gamma[i];
            let damping = -g * data.qvel[i];
            let noise = if active {
                let z: f64 = StandardNormal.sample(&mut *rng);
                (2.0 * g * self.k_b_t / h).sqrt() * z
            } else {
                0.0
            };
            qfrc_out[i] += damping + noise;
        }
    }

    fn as_stochastic(&self) -> Option<&dyn Stochastic> { Some(self) }
}

impl Stochastic for LangevinThermostat {
    fn set_stochastic_active(&self, active: bool) {
        self.stochastic_active.store(active, Ordering::Release);
    }
    fn is_stochastic_active(&self) -> bool {
        self.stochastic_active.load(Ordering::Acquire)
    }
}
```

`PassiveStack` exposes a one-shot toggle and an RAII guard:

```rust
// stack.rs

impl PassiveStack {
    /// Toggle the stochastic part of every stochastic component
    /// in the stack. Non-stochastic components are unaffected.
    /// Prefer `disable_stochastic()` when possible — the RAII
    /// guard cannot be forgotten.
    pub fn set_all_stochastic(&self, active: bool) {
        for c in &self.components {
            if let Some(s) = c.as_stochastic() {
                s.set_stochastic_active(active);
            }
        }
    }

    /// Disable every stochastic component for the lifetime of
    /// the returned guard. On drop, re-enables every component
    /// that was previously enabled. Use this around FD/autograd
    /// perturbation loops.
    pub fn disable_stochastic(&self) -> StochasticGuard<'_> {
        let prior: Vec<bool> = self.components.iter()
            .filter_map(|c| c.as_stochastic())
            .map(|s| {
                let was = s.is_stochastic_active();
                s.set_stochastic_active(false);
                was
            })
            .collect();
        StochasticGuard { stack: self, prior }
    }
}

pub struct StochasticGuard<'a> {
    stack: &'a PassiveStack,
    prior: Vec<bool>,
}

impl<'a> Drop for StochasticGuard<'a> {
    fn drop(&mut self) {
        for (c, &was) in self.stack.components.iter()
            .filter_map(|c| c.as_stochastic())
            .zip(self.prior.iter())
        {
            c.set_stochastic_active(was);
        }
    }
}
```

Use site (Phase 5 FD code):

```rust
let stack: Arc<PassiveStack> = ...;
// stack.install(&mut model) already done elsewhere

let baseline_qacc = {
    let _guard = stack.disable_stochastic();
    forward_skip(&model, &mut data);  // deterministic
    data.qacc.clone()
};

let mut grad = DVector::zeros(model.nv);
for i in 0..model.nv {
    let _guard = stack.disable_stochastic();
    data.qpos[i] += eps;
    forward_skip(&model, &mut data);
    grad[i] = (data.qacc[i] - baseline_qacc[i]) / eps;
    data.qpos[i] -= eps;
}
// guard drops, stochastic re-enabled, normal sampling resumes
```

**Pros**:
- **Solves the actual Phase 5 problem for the actual Phase 5 components.**
  All stochastic components on the roadmap have state-independent
  noise; disabling noise during FD gives exactly the right
  derivative.
- **Cheap.** One extension trait + one introspection hook + one
  RAII guard. ~40 LOC of chassis code total.
- **RAII makes forgetting impossible.** A user who reaches for
  `set_all_stochastic(false)` and forgets to call
  `set_all_stochastic(true)` leaves the system permanently
  deterministic. The guard makes that footgun unrepresentable.
- **Orthogonal trait pattern matches `Diagnose` (Decision 4).**
  Components opt in by implementing `Stochastic`; non-stochastic
  components ignore it entirely. The pattern is already
  established.
- **Composes with multi-stochastic stacks correctly.** A stack
  with three stochastic components and two deterministic ones
  toggles only the three. The two deterministic components are
  silently bypassed because their `as_stochastic` returns `None`.
- **Reentrant.** Nested guards (e.g., FD inside FD) compose
  correctly via the `prior: Vec<bool>` capture-and-restore
  pattern. Each guard restores to the state at its construction,
  not to "true."
- **Doesn't pollute non-stochastic components.** A user writing
  a custom drag-law `PassiveComponent` doesn't have to think
  about stochasticity at all — `as_stochastic` defaults to
  `None`, and no other change is required.

**Cons**:
- **Adds one method to `PassiveComponent`.** This is a small bleed
  of "what type of component is this?" introspection into the
  broad trait. Decision 1's "broad trait stays minimal" principle
  is mildly compromised, but the method has a sensible default
  and non-stochastic components pay zero cost.
- **Wrong for state-dependent noise.** A future component whose
  noise depends on `qpos`/`qvel` (e.g., multiplicative-noise
  processes) cannot be correctly handled by "just disable the
  noise" — it would also disable the *correct* state-dependent
  derivative contribution. Scheme B is required for that case.
  No such component is on the Research Directions roadmap, but
  it's worth being honest.
- **Atomic load per `apply()` call.** One `AtomicBool::load` per
  component per step. Negligible — atomics on x86 are nearly
  free in the uncontended case.

#### Scheme B — RNG snapshot/restore on `PassiveStack`

A more general mechanism: stochastic components expose
`rng_state(&self) -> Vec<u8>` and `restore_rng(&self, &[u8])`.
The stack collects per-component snapshots and restores them
around perturbation calls. The FD code does:

```rust
let snap = stack.snapshot_rng();
forward_skip(&model, &mut data);  // baseline, consumes RNG
let baseline_qacc = data.qacc.clone();
for i in 0..model.nv {
    stack.restore_rng(&snap);     // back to pre-baseline state
    data.qpos[i] += eps;
    forward_skip(&model, &mut data);  // perturbed, same noise sample
    grad[i] = (data.qacc[i] - baseline_qacc[i]) / eps;
    data.qpos[i] -= eps;
}
```

The perturbed and baseline runs see the *same* noise sample,
so the noise contribution cancels in the FD difference.

**Pros**:
- **Strictly more general.** Handles state-dependent noise
  correctly by ensuring that "the same noise sample appears in
  baseline and perturbed runs," not by "no noise appears in
  either."
- **No need for `Stochastic` to expose an `active` flag.** The
  noise still fires; it just fires from the same RNG state on
  every call.

**Cons**:
- **More implementation surface.** Each stochastic component
  must expose snapshot/restore for its RNG state. `ChaCha8Rng`
  state is ~40 bytes; serializing it correctly across `rand_chacha`
  versions is its own can of worms (the same hazard item 4
  resolved by *not* using `StdRng`). An explicit
  `serde`-ish dance per component is significant added complexity.
- **Per-call overhead is non-trivial.** Snapshot is a deep copy
  of RNG state; restore is a deep copy back. For an FD loop with
  `nv=1000` perturbations, that's 2000 RNG copies per gradient
  evaluation. Not catastrophic but measurable.
- **Doesn't compose with the `Mutex<ChaCha8Rng>` ownership model
  cleanly.** To snapshot, you'd lock the mutex, clone the
  internal state, unlock. To restore, lock, overwrite, unlock.
  Three lock acquisitions per FD call per stochastic component.
- **Harder to reason about.** A user reading the FD code has to
  understand that the perturbed and baseline runs are *meant* to
  produce identical noise — and if they accidentally call into
  unrelated code that draws from the RNG between snapshot and
  restore, the contract silently breaks.
- **Solves a problem we don't have on the roadmap.** No current
  or planned Research Direction needs state-dependent noise.

#### Recommendation: **Scheme A — orthogonal `Stochastic` trait + RAII guard**

Confidence: high. Five reasons in priority order:

1. **Correct for every stochastic component on the roadmap.**
   Langevin thermostat (Phase 1+), flashing ratchet (D1),
   stochastic resonance driver (D2), coupled bistable arrays
   (Phase 4), EBM training noise (Phase 5), Brownian computer
   lattices (D5) — all have state-independent noise. "Disable
   the noise" gives the exactly-correct derivative for all of
   them. Scheme B's extra generality is generality we don't
   need.

2. **RAII makes forgetting unrepresentable.** The same "loud
   over silent" principle that drove items 2, 4, 6, and M5:
   a user who forgets to re-enable noise gets a *visible*
   anomaly (their simulation is permanently deterministic) —
   bad, but obvious. Without the guard, the same forgetting
   gives *correct-looking but quietly wrong* statistics, which
   is the exact failure mode sharpen-the-axe forbids. Decision
   7 inherits the principle and applies it at the chassis API
   surface.

3. **~40 LOC vs. ~150 LOC.** Scheme A's total cost is
   `Stochastic` trait (~10 LOC) + `as_stochastic` default-None
   (~3 LOC) + `set_all_stochastic` (~8 LOC) + `disable_stochastic`
   + `StochasticGuard` (~25 LOC). Scheme B requires per-component
   serialization (~40 LOC × N components) + stack snapshot/restore
   (~50 LOC) + per-component lock dance (~20 LOC each). Scheme A
   is one quarter the surface for the same Phase 5 capability.

4. **Scheme B is additive on top of A, never the other way
   around.** If a future component lands with state-dependent
   noise, we add a *second* orthogonal trait `RngSnapshot:
   Stochastic` (or just `RngSnapshot: PassiveComponent`) and a
   second `PassiveStack::snapshot_rng_state` method. The
   `Stochastic` trait keeps working unchanged for everything
   that doesn't need snapshot/restore. The reverse is harder:
   if we ship Scheme B first and later realize most users only
   need "disable noise," we have a heavy API consumers must
   know how to wrap.

5. **Matches the chassis style.** Decision 4 ships `Diagnose`
   as an orthogonal opt-in trait. Decision 7 ships `Stochastic`
   as another orthogonal opt-in trait. The pattern is already
   established and proven; Scheme B would invent a new pattern
   (stack-level snapshot/restore) for one feature.

**Why I'm not recommending Scheme B**: it solves a problem we
don't have, at three to four times the implementation cost, with
a more error-prone use site. The "strictly more general" pro
turns into "more flexibility than the roadmap can use" in
practice. If a state-dependent noise component ever lands, we
add Scheme B alongside Scheme A — additive, non-breaking. The
chassis explicitly leaves that door open below.

#### Sub-decisions inside Decision 7

- **Trait name**: `Stochastic`. Matches the physics term, reads
  cleanly as `LangevinThermostat: PassiveComponent + Stochastic`.
  Considered: `Noisy` (informal), `RandomSource` (focuses on
  the source rather than the behavior), `HasNoise` (matches the
  Decision 4 forecast `HasTemperature` shape). `Stochastic` is
  the most idiomatic and the shortest.
- **Method names**: `set_stochastic_active` and
  `is_stochastic_active`. Verbose-on-purpose so they're hard to
  confuse with `Diagnose::diagnostic_*` methods or with
  generic `set_active`/`is_active` patterns from other crates.
- **Interior mutability via `AtomicBool`**: matches the existing
  `Mutex<ChaCha8Rng>` pattern in `LangevinThermostat`. Lighter
  than a mutex (no kernel call, no lock contention), correct
  for a single bool, `Send + Sync` for free. `Ordering::Acquire`
  for loads, `Ordering::Release` for stores — the standard
  release/acquire pattern for "publish a flag, then read it on
  another thread."
- **Default value**: `true` (stochastic-on by default). Set in
  `LangevinThermostat::new(...)`. A user who never touches it
  gets a fully-stochastic thermostat, which is the common case.
- **`as_stochastic` default returns `None`**: non-stochastic
  components opt in by overriding to return `Some(self)`. This
  is the only place `PassiveComponent` mentions `Stochastic`,
  and the bleed is justified by the cost of the alternative
  (separate builder methods, footgun risk — see "Why a single
  builder method" below).
- **Why a single builder method (`with`) instead of separate
  `with` and `with_stochastic`**: separate methods would let
  the user accidentally register a stochastic component with
  the non-stochastic builder method, silently breaking the
  gating. The introspection-hook approach makes the component
  itself responsible for declaring its stochasticity, removing
  the footgun. Decision 2's `with(impl PassiveComponent)`
  signature is unchanged.
- **`StochasticGuard` is `Drop`-based, not method-based**:
  RAII over `enable()`/`disable()` pairs because forgetting
  is the failure mode. The guard cannot be forgotten — when it
  goes out of scope, the disabled components are re-enabled.
  Reentrant by virtue of the `prior: Vec<bool>` capture (each
  guard restores to its construction-time state, not to
  hard-coded `true`).
- **`StochasticGuard` is non-`Send`** (no explicit `unsafe impl`,
  inherits from `&PassiveStack` which is `Send + Sync` so the
  guard is technically `Send`, but conceptually it should not
  cross thread boundaries because the disable/enable lifetime
  is scoped). The guard is intended for synchronous FD loops
  on a single thread; if Phase 5 ever needs cross-thread FD,
  we revisit.
- **Error mode if `Drop` panics**: `set_stochastic_active` is
  an `AtomicBool::store`, which is infallible. The guard's
  `Drop` implementation cannot panic. Clean.
- **`set_all_stochastic(false)` is exposed alongside the guard**:
  for the rare case where a user genuinely wants to leave
  stochasticity off for an indefinite period (e.g., debugging,
  golden-trajectory tests). Documented as "prefer the guard
  unless you have a specific reason."
- **No interaction with `install_per_env`**: each per-env stack
  has its own components and its own gating state. Toggling
  one stack's stochastic state does not affect any other stack.
- **Phase 1 test additions**: the Phase 1 test suite gains one
  small test that verifies `disable_stochastic` zeroes the noise
  contribution. Equipartition under disabled noise should give
  `⟨½v²⟩ → 0` (the deterministic damping decays the system to
  rest), which is a different but verifiable invariant.

#### Future direction (not Phase 1, named here for completeness)

If a future component lands with state-dependent noise (e.g., a
multiplicative-noise diffusion process), Scheme B can be added
as a second orthogonal trait *alongside* `Stochastic`:

```rust
pub trait RngSnapshot: PassiveComponent {
    type State: Clone + Send + Sync;
    fn snapshot_rng(&self) -> Self::State;
    fn restore_rng(&self, state: &Self::State);
}
```

Components that need snapshot/restore implement both
`Stochastic` (for the cheap "disable" path when correctness
allows it) and `RngSnapshot` (for the precise "same noise sample"
path when correctness requires it). The `PassiveStack` exposes
a parallel `snapshot_rng_states` / `restore_rng_states` API. None
of this changes Decision 7's `Stochastic` trait — it's purely
additive.

The chassis explicitly does not ship `RngSnapshot` in Phase 1
because no Phase 1-7 component needs it. If that changes, the
door is open.

#### DECISION (user confirmed): **Scheme A — `Stochastic` trait + RAII gating**

The chassis ships `Stochastic` as an orthogonal opt-in trait
with `set_stochastic_active` / `is_stochastic_active`,
`PassiveComponent` gains a default-`None` `as_stochastic`
introspection hook, and `PassiveStack` exposes both
`set_all_stochastic` and `disable_stochastic` (RAII guard) for
FD/autograd contexts. `LangevinThermostat` implements both
traits and gates its noise contribution on an `AtomicBool`.

Total chassis cost: ~40 LOC across `component.rs` (+~40 LOC)
and `stack.rs` (+~40 LOC).

Phase 5 FD code becomes:
```rust
let _guard = stack.disable_stochastic();
fd_perturbation_loop(...);
// guard drops, stochastic re-enabled
```

Scheme B (`RngSnapshot`) is named as the additive future
direction for state-dependent noise components, when and if
one lands.

#### Status

- **Decision 7 RESOLVED.** Scheme A confirmed. The "Phase 5+
  caveat" from master plan §The Gap
  is now answered at the chassis level instead of being
  punted; Phase 5 inherits a working FD path by construction.
- **Trait additions to Decision 1**: `PassiveComponent` gains a
  default-`None` `as_stochastic(&self) -> Option<&dyn Stochastic>`
  hook. The change is additive and non-breaking (every existing
  reference to `PassiveComponent` keeps working without
  modification).
- **API additions to Decision 2**: `PassiveStack` gains
  `set_all_stochastic(active)` and `disable_stochastic() ->
  StochasticGuard<'_>`. Both are pure additions; the existing
  `builder()` / `install()` / `install_per_env()` API is
  unchanged.
- **File inventory updated**: `component.rs` ~30 → ~70 LOC,
  `stack.rs` ~120 → ~160 LOC, total Phase 1 footprint ~810 →
  ~890 LOC.
- No code, no Cargo.toml changes, no new files written beyond
  this document.

---

## 2. Chassis design round complete

All seven decisions resolved:

| # | Decision                       | Resolution |
|---|--------------------------------|------------|
| 1 | Core trait shape               | `PassiveComponent` (broad, callback-shaped) |
| 2 | Composition idiom              | `PassiveStack::builder().with(...).build()` |
| 3 | Clone footgun resolution       | `install_per_env` + defensive clear + warnings |
| 4 | `Diagnose` trait surface       | Minimal: `diagnostic_summary -> String` only |
| 5 | Public test utilities          | `WelfordOnline` + `assert_within_n_sigma` + `sample_stats` |
| 6 | Crate layout                   | Flat ml-bridge style, 5 source files + 1 test file |
| 7 | Stochastic gating (FD/autograd)| `Stochastic` orthogonal trait + RAII `disable_stochastic()` guard |

The chassis is structurally complete. Phase 1 implementation
will be a learning exercise *for* the chassis (per the user's
"we might decide to redo the entire thing, as long as the bolt
patterns are the same, we can swap it" framing). If anything
about the trait shape, composer ergonomics, or test utilities
turns out wrong during Phase 1, the chassis is small enough
to revise without rewriting components.

### Open follow-ons before the Phase 1 spec

These came up during chassis design but are not chassis
decisions:

- **Documentation pass to the master plan**. Three things need
  updating:
  1. The recon log part 2's claim of "10⁻² sampling tolerance
     for 10⁵ samples" is incorrect due to autocorrelation
     (Decision 5 side finding). Effective N ≈ 10, not 10⁵.
  2. The recon log part 9 (item 7) flagged the clone footgun
     as needing Phase 1 spec resolution. Now resolved
     (Decision 3); the master plan should reference it.
  3. §The Gap Phase 1 needs the validation parameter set
     revised to reflect the autocorrelation finding. The
     three options are α (longer trajectories), β (multiple
     independent trajectories), γ (looser tolerance); the
     Phase 1 spec will choose, weak lean is β.
- **Phase 1 spec drafting**. With the chassis nailed down,
  the spec can be small and focused: implement
  `LangevinThermostat` against the trait, write the three
  Phase 1 tests using `test_utils`, choose the validation
  parameter fix, document the seed/RNG path. Estimated spec
  size: ~300-500 lines. Half-day of focused drafting.
