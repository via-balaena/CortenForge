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

Trait shape:

```rust
pub trait PassiveComponent: Send + Sync + 'static {
    fn apply(&self, model: &Model, data: &mut Data);
}
```

Introspection lives on the orthogonal `Diagnose` trait
(Decision 4). All future thermo-crate components — thermostats,
drivers, ratchets, custom drag laws, energy biases — implement
`PassiveComponent` and bolt to the same chassis.

#### Status

- **Decision 1 RESOLVED.** Scheme A confirmed. The trait shape
  above is the chassis for every component the thermo crate ever
  ships.
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
            for c in &me.components {
                c.apply(m, d);
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
    pub fn install(self: Arc<Self>, model: &mut Model) { ... }
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
