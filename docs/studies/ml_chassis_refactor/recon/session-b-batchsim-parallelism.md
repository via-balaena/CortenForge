# Recon session B — BatchSim parallelism audit

> **Status:** Raw recon, unverified. Do **not** lift text from this file
> directly into book chapters without first running the review protocol
> described in chapter 01 (factual pass against current source). Some
> file:line citations may have shifted since this was recorded.
>
> **When:** 2026-04-12, during the same session that committed chapters
> 00 and 01.
> **Method:** `Agent` tool with the `Explore` subagent, scoped prompt.
> **Used by:** Chapter 11 (BatchSim parallelism today), chapter 10
> (The wrong struct), chapter 14 (The new shape), chapter 15 (Design
> decisions). This recon provides the execution trace that chapter 11
> will render into prose and the design constraints that chapters 14/15
> will weigh.

## What the session asked

A precise account of how N parallel environments step in `BatchSim`,
where rayon enters (if it does), what `Data::reset` does, and what the
architectural seam looks like for moving stochastic state from `Model`
onto `Data`. The broader question was whether `BatchSim::step_all` is
actually parallel and what it costs to refactor.

## Key findings

### 1. BatchSim struct (reported)

`sim/L0/core/src/batch.rs:64-67` (reported):

```rust
pub struct BatchSim {
    model: Arc<Model>,
    envs: Vec<Data>,
}
```

Constructor `BatchSim::new(model: Arc<Model>, n: usize)` at
`batch.rs:73-76` allocates `Vec<Data>` by calling `model.make_data()`
N times. Each env has a fully independent `Data`. The `Arc<Model>` is
shared.

**No seeded variant.** There is no `BatchSim::new_seeded(..., seed: u64)`
today. The seed path currently goes through `LangevinThermostat::new`
before the thermostat is installed on the `Model`. If we move stochastic
state to `Data`, `BatchSim::new` becomes the natural place to seed
per-env RNGs — which means the constructor signature changes.

### 2. `step_all` IS parallel, gated on a feature flag

`sim/L0/core/src/batch.rs:148-167` (reported):

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

- **Parallel path:** `rayon::par_iter_mut()` on `Vec<Data>`.
  Splits on envs (one rayon task per env), not on DOF dimensions.
- **Sequential fallback:** when `parallel` feature is disabled, plain
  `iter_mut()`.
- **Feature definition:** `sim/L0/core/Cargo.toml` defines
  `parallel = ["dep:rayon"]`. Default-on? Needs verification.
- **No custom thread pool.** Rayon uses its default num_cpus pool.

**The non-determinism source is *specifically* lock acquisition
order inside the parallel region.** The ChaCha8 stream itself is
deterministic; the mutex serializes access; whichever env acquires
the lock first gets the next N_DOF samples. That "whichever" is
thread-scheduled and non-repeatable across runs.

### 3. Call chain from `step_all` down to `PassiveComponent::apply`

Agent B walked the full path (all citations reported, unverified):

1. `BatchSim::step_all` (`batch.rs:148`) → calls `data.step(model)`
2. `Data::step` (`forward/mod.rs:220`) → validates, routes to
   integrator, calls `self.forward(model)`
3. `Data::forward` (`forward/mod.rs:279-281`) → calls
   `self.forward_core(model, true)`
4. `Data::forward_core` (`forward/mod.rs:408-427`) → calls
   `self.forward_pos_vel` then `self.forward_acc`
5. `Data::forward_acc` (`forward/mod.rs:527-586`) → at line 535 calls
   `passive::mj_fwd_passive(model, self)`
6. `mj_fwd_passive` (`forward/passive.rs:348-736`) → zeros passive
   accumulators, computes springs/dampers/fluid forces/gravity
   compensation, then at line 719-721:
   ```rust
   if let Some(ref cb) = model.cb_passive {
       (cb.0)(model, data);
   }
   ```
7. `cb_passive` is the closure installed by `PassiveStack::install`.
   It iterates `self.components` and calls
   `component.apply(model, data_ref, &mut qfrc_out)`.
8. `LangevinThermostat::apply` at `thermostat/src/langevin.rs:141`
   is the leaf. At line 184 it locks `self.rng.lock()`, at 186-193
   it draws N_DOF Gaussian samples from the locked RNG, writes them
   into `qfrc_out`.

**Key point for the refactor:** `mj_fwd_passive` already holds
`&mut data`. The closure signature is `Fn(&Model, &mut Data)`. The
reason the trait method takes `&Data` is because `PassiveStack`'s
callback wrapper does the split-borrow with `mem::replace` to pull
`qfrc_passive` out of `Data` as a separate `&mut DVector<f64>`.
**There's no fundamental obstacle to passing `&mut Data` through
to the trait method — it's a design choice in `PassiveStack`, not
a borrow-checker constraint.**

This matters because the "RNG on Data" refactor is often framed as
requiring `unsafe` or interior mutability. It doesn't. The real cost
is updating the trait signature and rewriting `PassiveStack`'s
split-borrow dance.

### 4. `Data::reset` has NO RNG state today

Agent B reported `Data::reset` at `data.rs:1064-1200` (unverified).
The function resets:
- `qpos`, `qvel`, `qacc`, `qacc_warmstart`
- All control: `ctrl`, `act`, `act_dot`, `qfrc_actuator`, `actuator_*`
- All forces: `qfrc_passive`, `qfrc_spring`, `qfrc_damper`,
  `qfrc_gravcomp`, `qfrc_fluid`, `qfrc_constraint`
- Contacts, sensors, energy, warnings, sleep state, island state,
  mocap

**What it doesn't touch:** there is no RNG field on `Data` today.
Adding one is safe from a reset perspective — the new field would
either be preserved (default) or re-seeded (via `reset_with_seed`).

**No `reset_with_seed` variant exists.** If we want episode-level
re-seeding (for reproducibility across episodes), we'd add it.

### 5. `Data` struct characteristics (reported)

- `#[derive(Debug)]` (line 31)
- Has a **manual `Clone` impl** (line 671) that clones every field
  by hand (~100 fields). Adding an `rng: ChaCha8Rng` field would
  require one line in that impl. `ChaCha8Rng` does derive `Clone`.
- No `Copy` derive (so adding a non-Copy field is safe)
- No `Serialize`/`Deserialize` currently — if that ever gets added,
  `ChaCha8Rng` would need a serde wrapper

### 6. `VecEnv::step` dispatch (reported)

`sim/L0/ml-bridge/src/vec_env.rs:132-248`:

- Line 156: `let errors = self.batch.step_all()` — direct dispatch
  to `BatchSim::step_all`, which means the parallel region happens
  inside the VecEnv step.
- Sub-stepping loop at 155-167: `for _ in 0..self.sub_steps {
  let errors = self.batch.step_all(); ... }`. All sub-steps run for
  all envs in a single `step_all()` call. No intra-step parallelism.
- Lines 194-226: sequential auto-reset and observation extraction
  runs *after* `step_all()` returns. The passive callback
  (including Langevin noise draws) happens *inside* `step_all()`,
  in the parallel region.

### 7. Existing reproducibility tests

`sim/L0/tests/integration/batch_sim.rs` has these tests (reported):

- **`batch_matches_sequential_with_contacts`** (lines 54-91) — asserts
  bit-exact equality between parallel and sequential stepping for
  N=4 envs. **But only uses deterministic physics** (contact model,
  no thermostat). This is the closest thing to a parallel-determinism
  test, and it passes today because stochastic components aren't
  installed.
- `nan_auto_reset_with_mjcf_model` — unrelated, checks NaN handling
- `per_env_ctrl_produces_different_trajectories` — unrelated, checks
  per-env ctrl routing

Agent B confirmed: **no test asserts that N parallel environments
stepped via `BatchSim::step_all()` with stochastic components produce
bit-exact results independent of thread scheduling.** The determinism
invariant exists in the test suite's intent (for deterministic
physics) but silently breaks the moment a `LangevinThermostat` gets
installed.

### 8. Cost to move stochastic state onto `Data`

Agent B's summary of the refactor cost:

**Required changes:**
- `Data` gains `rng: ChaCha8Rng` field (one line struct, one line
  in manual Clone impl)
- `BatchSim::new` gains a `seed: u64` parameter, seeds each env's
  RNG via `seed ^ env_index` or similar
- `PassiveComponent::apply` signature changes from `(&self, &Model,
  &Data, &mut DVector<f64>)` to `(&self, &Model, &mut Data, &mut
  DVector<f64>)` — or some equivalent that gives the method mutable
  access to `Data`
- `PassiveStack::install` wrapper closure rewrites the split-borrow
  to pass `&mut data` through to the trait method instead of the
  current `&data + &mut qfrc_passive` split
- `LangevinThermostat` loses its `rng: Mutex<ChaCha8Rng>` field and
  its `seed: u64` field; the `apply` implementation reads
  `data.rng` directly instead of locking
- Every other `PassiveComponent` implementor gets updated to the
  new signature (all as no-ops: `_data: &mut Data`)
- Tests that construct `LangevinThermostat::new(γ, kT, seed)` get
  updated to `LangevinThermostat::new(γ, kT)` and pass the seed to
  `BatchSim::new` instead

**Obstacle count:** 0 (nothing fundamentally blocks this)
**Rippling count:** medium (trait signature change touches every
component, every test using the trait, and `PassiveStack`'s
callback-wrapping machinery)

**Estimated effort:** 1-2 days of careful work. Most of the cost is
signature propagation and test updates, not logic.

## My synthesis notes

Things I noticed while reading the agent's report:

1. **The refactor is genuinely bounded.** Agent B confirms there's
   no deep obstacle — it's a signature change that ripples through
   known call sites. The "this requires unsafe" and "this requires
   interior mutability on Data" framings are wrong. Plain
   `&mut Data` in the trait method works once `PassiveStack`'s
   callback wrapper is updated.

2. **The gating test (from session A) is the tightest constraint.**
   `apply_does_not_advance_rng_when_stochastic_inactive` asserts
   that deterministic-mode calls don't consume RNG state. If state
   moves to `Data`, the "don't consume" check has to work per-env,
   which means the `Stochastic` trait's gating flag also needs to
   be addressable at the `Data` level or via interior mutability
   on the thermostat struct. Worth thinking through in chapter 15.

3. **Per-env seed derivation is a real design call.** `seed ^ env_index`
   has a bit-flip failure mode (seed=0, env_index=0 gives env_seed=0
   — collides with a different master). Better options: splitmix64,
   or `rand`'s built-in stream hierarchy, or `ChaCha8Rng::from_u64(master).next_u64() + env * stride`.
   One-evening design call that deserves being made explicitly, not
   left to a TODO.

4. **The regression test for "did we fix it" needs to be written
   before the refactor.** A new test like
   `batch_matches_sequential_under_langevin` that installs a
   thermostat, steps N parallel envs, and asserts reproducibility
   across runs with the same master seed. Without this test, "is
   it fixed" has no automated answer. Chapter 15 should spec this.

5. **The existing bit-exact test `batch_matches_sequential_with_contacts`
   is NOT broken** — it just doesn't cover stochastic physics. No
   regression there. But its existence tells us the project already
   values parallel-sequential determinism as an invariant, which
   strengthens the case for the refactor.

## Caveats and verification status

- **Every file:line citation in this document is agent-reported,
  not verified.** Factual pass is mandatory before chapter use.
- **`#[cfg(feature = "parallel")]` enablement in the default build
  is unverified.** Agent B didn't check whether the feature is
  default-on in the workspace. Worth confirming before claiming
  "parallel is engaged by default."
- **The 1-2 day effort estimate is a guess.** Could be larger if
  test surfaces are bigger than expected or if the split-borrow
  rewrite hits unforeseen issues. Not a commitment, just a rough
  bucket.

## What Phase 2 chapter drafts need from this file

- **Chapter 11 (BatchSim parallelism today):** the call chain,
  the feature gate, the mutex-as-serialization finding, the
  existing determinism test that's scoped to deterministic physics
  only.
- **Chapter 10 (The wrong struct):** the physics argument can cite
  this file's demonstration that `Data` has no RNG today and
  `Model` holds it via `LangevinThermostat` behind a mutex.
- **Chapter 13 (Latent flakiness):** the absence of a parallel-
  stochastic-determinism test is the marquee finding.
- **Chapter 14 (The new shape):** the trait signature change, the
  `PassiveStack` rewrite, the per-env seed injection point in
  `BatchSim::new`, the 1-2 day effort estimate.
- **Chapter 15 (Design decisions):** the per-env seed derivation
  call, the `Stochastic` gating preservation, the regression test
  spec.
