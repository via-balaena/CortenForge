# Recon session A — Stochastic component survey

> **Status:** Raw recon, unverified. Do **not** lift text from this file
> directly into book chapters without first running the review protocol
> described in chapter 01 (factual pass against current source). Some
> file:line citations may have shifted since this was recorded.
>
> **When:** 2026-04-12, during the same session that committed chapters
> 00 and 01.
> **Method:** `Agent` tool with the `Explore` subagent, scoped prompt.
> **Used by:** Chapter 12 (Stochastic component survey), chapter 10
> (The wrong struct), chapter 14 (The new shape), chapter 15 (Design
> decisions). This recon provides the concrete "what's where" that
> those chapters analyze and act on.

## What the session asked

A systematic survey of every passive-force component in
`sim/L0/thermostat/src/` to produce a structured report on where each
one stores stochastic state and how it's seeded. The broader question
was whether `LangevinThermostat` is the *only* stochastic component
(meaning the refactor is scoped to one file) or whether other components
have the same mutex-on-shared-Arc pattern (meaning the refactor is
trait-wide).

## Key findings

### 1. Only `LangevinThermostat` holds RNG state

Of the components surveyed, **exactly one** stores stochastic state:

| Component | RNG? | Location | Seed source |
|---|---|---|---|
| `LangevinThermostat` | **yes** | `Mutex<ChaCha8Rng>` field on the struct, installed on shared `Arc<Model>` | constructor `u64` param, retained as `seed: u64` field for `diagnostic_summary` |
| `DoubleWellPotential` | no | — | — |
| `OscillatingField` | no | — | — |
| `RatchetPotential` | no | — | — |
| `PairwiseCoupling` | no | — | — |
| `ExternalField` | no | — | — |

Reported citations (all `sim/L0/thermostat/src/`, unverified):

- `langevin.rs:72` — struct definition
- `langevin.rs:75` — `seed: u64` field
- `langevin.rs:76` — `rng: Mutex<ChaCha8Rng>` field
- `langevin.rs:102` — `LangevinThermostat::new(gamma, k_b_t, seed)`
- `langevin.rs:107` — `ChaCha8Rng::seed_from_u64(seed)` inside `new`
- `langevin.rs:141` — `impl PassiveComponent::apply(&self, &Model, &Data, &mut DVector<f64>)`
- `langevin.rs:184` — `self.rng.lock()` inside apply
- `langevin.rs:186-193` — noise sample loop
- `langevin.rs:211-233` — `Diagnose::diagnostic_summary()` formatting seed into string

**Scope implication:** the refactor targets one component, not a trait
hierarchy redesign. Other components accept the new `PassiveComponent`
signature (whatever it becomes) as no-ops.

### 2. The `Stochastic` sibling trait exists and is load-bearing

`sim/L0/thermostat/src/component.rs:101-108` (reported):

```rust
pub trait Stochastic: Send + Sync {
    fn set_stochastic_active(&self, active: bool);
    fn is_stochastic_active(&self) -> bool;
}
```

This is Decision-7 gating. A stochastic component implements *both*
`PassiveComponent` and `Stochastic`. When `set_stochastic_active(false)`
is called, the component must produce only deterministic forces (e.g.,
Langevin's `−γ·v` damping) without drawing any noise. When `true`, full
stochastic contribution. Documented via interior-mutability flag on the
component.

**Load-bearing test:** `langevin.rs:312-363` has
`apply_does_not_advance_rng_when_stochastic_inactive` that runs two
consecutive applies with stochastic disabled and verifies the RNG state
did not advance. Any refactor that moves RNG state to `Data` must
preserve this invariant. The way to do that cleanly is non-obvious —
if stochastic state lives per-env on `Data`, the "skip RNG advance in
deterministic mode" check has to work per-env too.

This is a **chapter 14/15 design constraint** that the straightforward
"just move RNG to Data" framing doesn't address.

### 3. `PassiveComponent::apply` signature (reported)

`sim/L0/thermostat/src/component.rs:61-75`:

```rust
pub trait PassiveComponent: Send + Sync + 'static {
    fn apply(&self, model: &Model, data: &Data, qfrc_out: &mut DVector<f64>);
    fn as_stochastic(&self) -> Option<&dyn Stochastic> {
        None
    }
}
```

**Key observation:** `data: &Data` is immutable. The M5 contract
explicitly says components read model and data immutably. This is the
seam that makes the proposed refactor non-trivial: moving RNG to
`Data` means either (a) signature change to `&mut Data`, which ripples
through the callback-wrapping split-borrow in `PassiveStack`, or (b)
interior mutability on `Data` (Cell or Mutex), which re-creates the
problem we're trying to solve.

### 4. `PassiveStack` storage pattern (reported)

`sim/L0/thermostat/src/stack.rs:100-136`:

```rust
pub struct PassiveStack {
    components: Vec<Arc<dyn PassiveComponent>>,
}
```

Install via `install(self: &Arc<Self>, model: &mut Model)` which sets
`model.cb_passive` to a closure that iterates `self.components` and
calls `apply` on each. The closure uses `std::mem::replace` on
`data.qfrc_passive` as a split-borrow trick (temporarily takes the
forces vector out of `Data`, passes it as `qfrc_out`, puts it back).
This is why `apply` takes `&Data`: the split-borrow lets the trait
method be read-only on `Data` while the closure holds `&mut Data`.

### 5. `install_per_env` exists (reported but needs verification)

Agent A mentioned a function at `stack.rs:174-199` that creates N
independent `(Model, Arc<PassiveStack>)` pairs via a factory. Each pair
has its own stack with its own component instances and RNG state. This
would be an alternative refactor path: instead of moving RNG onto
`Data`, keep the per-component architecture but make each env own its
own `Model` + `PassiveStack`.

**Cost of `install_per_env` path:** `VecEnv`/`BatchSim` currently holds
a single `Arc<Model>` shared across all envs. Switching to per-env
models means `BatchSim` has to become `Vec<(Arc<Model>, BatchEnv)>`
or similar — fundamentally different architecture, probably more
invasive than the "RNG on Data" path but with a different risk
profile.

**This is the Option A / Option B fork** I flagged for user review in
chapter 14/15. Needs source verification before making the design call.

### 6. `GibbsSampler` is NOT a `PassiveComponent`

Reported at `sim/L0/thermostat/src/gibbs.rs:26-147`. Standalone
discrete-spin sampler for Ising models. Owns its own `ChaCha8Rng`
directly (not behind a mutex). Has mutable `sweep(&mut self)` method.
Called from test code as a reference sampler, not from any passive
stack.

**Implication for the pivot's "Gibbs beats Langevin" claim:** the
comparison is valid but apples-to-oranges in implementation terms.
Gibbs is a standalone sampler with clean per-instance RNG; Langevin
is a passive component with shared-Arc RNG. The architectural
asymmetry is itself part of why Gibbs works cleanly and Langevin
doesn't.

## My synthesis notes (not from the agent directly)

Things I noticed while reading the agent's report:

1. **Only one component is stochastic → refactor is scoped, not
   trait-wide.** This is the biggest load-reducer for the refactor plan.
   The naïve framing "rewrite all passive components" was wrong; the
   reality is "fix the one stochastic component and everything else
   stays."

2. **The `Stochastic` sibling trait + the `apply_does_not_advance_rng`
   test is a non-obvious refactor constraint.** The "RNG on Data"
   design has to preserve the gating behavior. Naively moving state
   to `Data` breaks the "stochastic-inactive doesn't advance" guarantee
   unless the gating is also moved to `Data`.

3. **`install_per_env` path deserves serious consideration.** Agent A
   mentioned it as an alternative. Worth investigating before
   committing to Option A ("RNG on Data"). Question for chapter 15:
   which design has the smaller blast radius, and which is more
   honest about where state lives?

4. **`sim/L0/tests/integration/batch_sim.rs`'s
   `batch_matches_sequential_with_contacts` test asserts bit-exact
   equality between parallel and sequential stepping for N=4 envs —
   but only for deterministic physics.** There's no equivalent test
   for stochastic components. Chapter 13 (latent flakiness) should
   flag this: the determinism invariant exists in the test suite's
   intent but breaks silently as soon as anyone installs a
   `LangevinThermostat`. Adding such a test is a prerequisite for
   the refactor — otherwise "did we fix it" has no objective answer.

## Caveats and verification status

- **Every file:line citation in this document is agent-reported, not
  verified.** Before any of these lands in a chapter, the factual pass
  (chapter 01 §2) must confirm the line numbers against current source.
- **The `install_per_env` claim at `stack.rs:174-199` is unverified.**
  Needs a direct `Read` of that file to confirm the function exists,
  what its signature is, and how it integrates with `BatchSim`.
- **The `Stochastic` trait's exact signature at `component.rs:101-108`
  is unverified.** Agent A reproduced the trait body but I should
  confirm it matches what's actually in the file before citing the
  specific methods.
- **`apply_does_not_advance_rng_when_stochastic_inactive` at
  `langevin.rs:312-363` is a test name and line range agent A
  reported.** Should be grep-verified.

## What Phase 2 chapter drafts need from this file

- **Chapter 12 (Stochastic component survey):** the full component
  table, the `Stochastic` trait explanation, the `install_per_env`
  finding, the Gibbs-is-not-a-PassiveComponent note.
- **Chapter 10 (The wrong struct):** the physics argument is
  independent of this recon, but the chapter can cite this file as
  concrete evidence that "only one component is wrong."
- **Chapter 14 (The new shape):** the M5 contract obstacle, the
  `Stochastic` gating constraint, the install_per_env alternative.
- **Chapter 15 (Design decisions):** the per-env seed derivation
  function, which design option wins, how the gating is preserved.
