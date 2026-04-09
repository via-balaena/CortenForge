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
