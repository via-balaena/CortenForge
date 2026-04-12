# The Physics-Aware ML Pivot

> **Status: reviewed 2026-04-12.** Every concrete claim in this document has
> been cross-checked against the source tree. Execute the plan in §"Execution
> plan" as written.

## One-paragraph summary

CortenForge has been building generic RL infrastructure (currently
`sim-ml-bridge`: CEM, REINFORCE, PPO, TD3, SAC) and applying it to generic
robotic tasks (2-DOF and 6-DOF reaching arms). The generic work is fine as a
baseline and onboarding layer, but it isn't the mission. The mission is
**custom, physics-aware optimization and sampling for thermodynamic
computing**, and the evidence is concrete: (a) 6-DOF visual examples look
mediocre because generic linear-CEM is poorly fit to problems with strong
physical structure, (b) a 1984 algorithm (Gibbs sampling) in
`sim-thermostat` strictly beats Langevin dynamics on Ising sampling at both
the accuracy and the work gate, and (c) D2c showed CEM, TD3, PPO, and SAC
all failing on stochastic resonance with linear function approximation —
physics-aware baselines would have had an obvious shot. The fix is a
three-crate restructure of the algorithm side of CortenForge: split
`sim-ml-bridge` into **`sim-ml-chassis`** (shared primitives, traits,
networks, Competition runner — the foundation) and **`sim-rl`** (just the
RL baselines: CEM, REINFORCE, PPO, TD3, SAC), then add a new sibling
**`sim-opt`** containing physics-aware algorithms (Simulated Annealing,
Parallel Tempering, CMA-ES, Metropolis-Hastings) that compete head-to-head
against the RL baselines on every thermo problem using the Competition
runner from `sim-ml-chassis`.

## Two corners (the mission)

CortenForge's real research direction is a **self-improving loop** between
two domains that feed each other:

**Corner 1 — Custom, physics-aware optimization / sampling.**
Not deep RL. Population-based search, MCMC, replica exchange, evolutionary
strategies — algorithms whose structure *matches the physics of
thermodynamic computers*. Each algorithm is a thin wrapper over physics
primitives (Langevin, Gibbs, Metropolis). Domain knowledge is injected via
the update rule, not learned from scratch.

**Corner 2 — Thermodynamic computing hardware.**
Mechanical bistable elements — buckling beams, snap-through latches,
Brownian ratchets — whose probabilistic behavior **is** the computation.
See `docs/thermo_computing/01_vision/vision.md`.

**How they close the loop:**

- Thermo computers get cheaper/faster to manufacture → more training data → better physics-aware optimization.
- Better physics-aware optimization designs better thermo-computer architectures and factory processes → better thermo computers.
- Critically: **thermo computers are themselves a substrate for ML computation.** At some point the loop eats itself — physics-aware algorithms running on thermo computers designing better thermo computers.

## The hockey skate principle

Off-the-shelf RL (PPO, SAC, TD3) applied to generic tasks is a best-in-class
retail hockey skate. It will never compete with a properly custom pair.
Custom skates require deep domain knowledge and specific intent — nobody
else has the incentive to build them because nobody else is solving the
same problem. For CortenForge, "custom skate" means algorithms that
understand the physics of thermodynamic computing: energy-aware exploration,
thermodynamic-constraint-respecting updates, design primitives that map to
manufacturable thermo-computing elements, reward shapes that reflect real
fabrication economics.

## Evidence the pivot is right

**1. 6-DOF generic CEM-linear looks bad because it is a bad fit.**
CEM + linear policy on 6-DOF reaching has 78 parameters in a search space
with broad plateaus and saddle points. Learning happens (reward improves
~95%), but the visual is dominated by random chaos because the algorithm
has zero domain knowledge. The 2-DOF version (10 params) looks good because
the search space is small enough for blind sampling to converge cleanly.
Visual quality tracks how well the algorithm fits the problem structure.

**2. `reset-subset` (sim-cpu batch-sim example) visually outperforms
CEM-linear on 6-DOF.**
Twelve landers, each with a scalar thrust parameter. The update rule is
"if crashed, increase thrust; if hovering, decrease thrust." This is 1D
parameter search with a hand-crafted, domain-aware update. It converges in
~10 seconds and looks beautiful. It's not RL at all. It's a demonstration
that **population-based search with hand-crafted update rules often beats
generic RL on practical problems**.

**3. Gibbs sampling strictly beats Langevin on the Ising task.**
`sim/L0/thermostat/tests/gibbs_sampler.rs` Gate B (lines 270–287) asserts:

```rust
assert!(tv_gibbs_exact    < 0.02);
assert!(tv_langevin_exact < 0.15);
assert!(tv_gibbs_exact    < tv_langevin_exact);
```

Gate B's work budgets (from the same file, lines 57–64):

| Sampler | Accuracy gate (TV from exact) | Work budget |
|---|---|---|
| Gibbs (Geman-Geman 1984) | < 0.02 | 100K samples × 4-site sweeps + 1K burn-in |
| Langevin (physical thermostat) | < 0.15 | 10 trajectories × 5M steps |

These are test-gate thresholds, not measured wall-clock numbers. The real
claim is the *strict* comparison in the third assert: Gibbs beats Langevin
on the same problem, with an accuracy budget that is **tighter by more than
7× and a work envelope roughly two orders of magnitude smaller** (site
updates and integration steps are not the same units, so we do not claim a
specific multiplier until we have matched wall-clock). The qualitative
point is solid: a 1984 algorithm, purpose-built for sampling multivariate
Boltzmann distributions, strictly outperforms the heavier "physical"
simulation because it exploits the conditional structure of the distribution.

**4. D2c is the first competition — and it showed generic RL failing.**
`sim/L0/thermostat/tests/d2c_cem_training.rs` ran CEM, TD3, PPO, and SAC
against stochastic resonance with **linear** function approximation
(LinearPolicy / LinearQ / LinearValue / LinearStochasticPolicy), 100 epochs,
5000-step episodes, 32 parallel envs. Results (`project_d2_sr_findings.md`):

| Algorithm | Eval kT | Gate A | Finding |
|-----------|---------|--------|---------|
| CEM | 0.99 | FAIL | Finds SR band, can't resolve peak |
| TD3 | −0.31 | FAIL | Linear Q insufficient |
| PPO | 0.005 | PASS\* | Exploration-noise inflation (not SR) |
| SAC | −0.78 | FAIL | Linear Q insufficient |

\* False positive — synchrony came from transient exploration noise, not
from discovered resonance.

**This is exactly the experiment to rematch.** Adding simulated annealing +
parallel tempering to the same pool would answer "does physics-aware win
here" directly. Framing matters: the rematch is *physics-aware vs
matched-complexity (linear) RL*. An MLP/autograd RL rematch is a
legitimate follow-up, but not the first test — the first test is whether
physics structure beats matched-expressiveness RL on a problem designed
around physics structure.

## The algorithm family to build

All pre-date modern deep RL, all are well-understood, all have strong
connections to thermodynamics:

| Algorithm | Year | Attribution | What it does |
|---|---|---|---|
| Metropolis-Hastings | 1953 / 1970 | Metropolis et al. / Hastings | MCMC accept/reject — the foundation |
| **Simulated Annealing** | **1983** | Kirkpatrick, Gelatt, Vecchi (Science) | Cooling schedule for global optimization |
| **Gibbs Sampling** | **1984** | Geman & Geman (IEEE TPAMI) | Conditional MCMC — already in `sim-thermostat` |
| **Boltzmann Machine Learning** | **1985** | Ackley, Hinton, Sejnowski | Learning with Gibbs sampling |
| **Parallel Tempering / Replica Exchange** | **1991 / 1996** | Geyer (1991); Hukushima & Nemoto (1996) | Parallel chains at different temperatures, swap states |
| CMA-ES | 1996 / 2001 | Hansen & Ostermeier | Covariance-matrix adaptation evolution strategy |
| Differential Evolution | 1995 | Storn & Price | Population-based evolutionary search |

Parallel Tempering is the standout for thermo-RL: parallel chains at
different temperatures that swap states is *literally* what a
thermodynamic computer would naturally implement in hardware. (Swendsen
& Wang 1986 is the cluster algorithm — a related but distinct method; not
the source of modern PT.)

### Why not depend on `argmin` / `cmaes` / existing crates?

`argmin` (generic optimization) and `cmaes` exist on crates.io and are
mature. We write our own anyway for three specific reasons:

1. **Tight integration** — every algorithm has to compose `VecEnv`,
   `Algorithm::train()`, `Policy`, `TaskConfig`, and `Competition` out of
   the box. Adapters around external crates would fight the chassis.
2. **Short code** — each algorithm in this family is a few hundred lines.
   The cost of writing them is lower than the cost of making `argmin`'s
   cost/gradient model play nice with an episodic env.
3. **The whole point is custom-skate ownership** (§"The hockey skate
   principle"). An external dependency is a retail skate with a
   CortenForge sticker on it.

This is a deliberate rejection, not an unaware one.

## The crate structure (decided)

Three crates for the algorithm side of CortenForge. The existing
`sim-ml-bridge` is split into `sim-ml-chassis` (shared foundation) and
`sim-rl` (RL baselines). A new `sim-opt` sibling crate is added for
physics-aware algorithms. Both `sim-rl` and `sim-opt` depend on
`sim-ml-chassis` — they are siblings, not parent-child.

```
sim/L0/
├── core/                ← physics chassis (existing, unchanged)
├── thermostat/          ← physics primitives (existing, unchanged)
│   ├── LangevinThermostat, GibbsSampler (Ising-specific instance — stays here)
│   ├── DoubleWell, Ratchet, Ising, PairwiseCoupling, ExternalField
│   └── PassiveStack
│
├── ml-chassis/          ← NEW: the algorithm chassis (from split)
│   ├── Primitives: VecEnv, SimEnv, Tensor, ReplayBuffer, Optimizer,
│   │               autograd, Rollout, GAE, PolicyArtifact,
│   │               TrainingCheckpoint, BestTracker, Competition
│   ├── Traits: Algorithm, Policy, DifferentiablePolicy,
│   │           StochasticPolicy, ValueFn, QFunction, Environment
│   ├── Errors: EnvError, ResetError, SpaceError, TensorError,
│   │           VecStepError, ArtifactError
│   ├── Networks: Linear/Mlp/Autograd × (Policy, Value, Q)
│   └── Tasks: TaskConfig, TaskConfigBuilder, reaching_2dof,
│              reaching_6dof, obstacle_reaching_6dof
│
├── rl/                  ← NEW (from split): RL baselines only
│   ├── Cem, Reinforce, Ppo, Td3, Sac
│   └── Re-exports: Algorithm, VecEnv, Policy, Tensor, TrainingBudget
│
└── opt/                 ← NEW: physics-aware optimization
    ├── annealing.rs     — Simulated Annealing (Kirkpatrick 1983)
    ├── tempering.rs     — Parallel Tempering (Geyer 1991 / Hukushima 1996)
    ├── metropolis.rs    — General Metropolis-Hastings wrapper
    ├── cma_es.rs        — CMA-ES (Hansen & Ostermeier 1996/2001)
    ├── de.rs            — Differential Evolution (optional, later)
    └── lib.rs           — shared helpers, competition builders
```

### Why "chassis"?

`sim-ml-chassis` slots into the **R34 GT-R architecture principle**
(`feedback_r34_architecture.md`): overbuild the chassis, bolt on
aftermarket.

- `sim-core` is the **physics chassis** (rigid-body dynamics, constraints).
- `sim-ml-chassis` is the **algorithm chassis** (environments, policies,
  traits, primitives).
- `sim-rl` is an aftermarket family (RL baselines) bolted onto ml-chassis.
- `sim-opt` is an aftermarket family (physics-aware algorithms) bolted
  onto ml-chassis.
- Future families (bio-inspired control, imitation learning, etc.) also
  bolt onto ml-chassis without touching rl or opt.

Every new algorithm family is a new aftermarket crate, not a modification
of the chassis. This is the agility test (`feedback_agility_test.md`) made
architectural.

### Why three crates, not two

Leaving `sim-ml-bridge` unified (primitives + RL baselines) and renaming
it `sim-rl` would force `sim-opt` to depend on the RL baselines crate for
primitives, creating a false parent-child relationship where RL looks more
foundational than opt. They are supposed to be **peers**.

```
           sim-ml-chassis
             /        \
         sim-rl      sim-opt
         (RL algos)  (physics-aware algos)
```

Neither crate depends on the other at runtime. Both build on the shared
foundation. Competition runs them head-to-head as equals. Adding a third
algorithm family later is symmetric to how `sim-rl` and `sim-opt` relate
today — no special-casing required.

**Cost of the extra crate:** one extra `Cargo.toml`, one extra line in the
workspace manifest, one extra grading target, and for consumers that mix
chassis types with algorithms, import paths split across two crate names
(mitigated by `sim-rl` re-exporting the common chassis types). That cost
is paid once; the symmetry is paid forward forever.

### The split (sim-ml-bridge → sim-ml-chassis + sim-rl)

No code is rewritten. Files are moved:

**To `sim-ml-chassis`:**
- Traits: `algorithm.rs`, `policy.rs`, `value.rs`, `env.rs`
- Infra: `vec_env.rs`, `tensor.rs`, `space.rs`, `optimizer.rs`,
  `replay_buffer.rs`, `rollout.rs`, `gae.rs`
- Autograd: `autograd.rs`, `autograd_layers.rs`, `autograd_policy.rs`,
  `autograd_value.rs`
- Networks: `linear.rs`, `mlp.rs`
- Artifacts: `artifact.rs`, `best_tracker.rs` (promoted from `pub(crate)` to `pub`)
- Errors: `error.rs` (EnvError, ResetError, SpaceError, TensorError, VecStepError)
- Runner: `competition.rs`
- Tasks: `task.rs` (TaskConfig + stock factories — see rationale below)
- Integration tests: `tests/custom_task.rs`, `tests/competition.rs`
- Benches: `benches/bridge_benchmarks.rs`

**To `sim-rl`:**
- Algorithms: `cem.rs`, `reinforce.rs`, `ppo.rs`, `td3.rs`, `sac.rs`
- Re-exports: `Algorithm`, `VecEnv`, `Policy`, `DifferentiablePolicy`,
  `StochasticPolicy`, `Tensor`, `TrainingBudget`, `EpochMetrics` — so
  `use sim_rl::{Cem, Algorithm, VecEnv}` still works without touching
  `sim_ml_chassis::` directly.

**Why `task.rs` stays in chassis (all of it):** `TaskConfig` and
`TaskConfigBuilder` are obviously chassis-level primitives. The three stock
factories (`reaching_2dof`, `reaching_6dof`, `obstacle_reaching_6dof`) also
stay in chassis because the chassis's own unit tests
(`competition.rs:429,561,695` in current `sim-ml-bridge`) reference them.
Moving them out would break the chassis's tests.

**Where Gibbs lives — closed.** `GibbsSampler` stays in `sim-thermostat` as
a concrete Ising instance. `sim-opt` provides a general Metropolis-Hastings
wrapper and, where useful for competition tests, depends on `sim-thermostat`
to access the specific instance. This keeps the dependency direction clean:
physics-specific solvers live with the physics; general optimization
algorithms live in `sim-opt`. Option 2 (move Gibbs out of thermostat) is
rejected — it would invert the direction and force every consumer of
`sim-thermostat` to pull in `sim-opt`.

### Import sites and example classification

I grepped `use sim_ml_bridge::` across the tree. Classification post-split:

**Chassis-only** (no algorithm imports — these just need the path rewritten
to `sim_ml_chassis`):

- `examples/fundamentals/sim-ml/spaces/{obs-rich, obs-extract, act-inject, act-clamping, stress-test}`
- `examples/fundamentals/sim-ml/sim-env/{sub-stepping, episode-loop, on-reset, stress-test}`
- `examples/fundamentals/sim-ml/vec-env/{parallel-step, terminal-obs, stress-test, auto-reset}`
- `examples/fundamentals/sim-ml/6dof/stress-test` (uses `LinearPolicy`/`MlpPolicy`/`AutogradStochasticPolicy` — all chassis types)
- `examples/fundamentals/sim-ml/shared`

**sim-rl consumers** (need both `sim_ml_chassis` and `sim_rl`, or `sim_rl`
alone via re-exports):

- `examples/fundamentals/sim-ml/vec-env/{ppo, sac, td3, reinforce}`
- `examples/fundamentals/sim-ml/6dof/cem-linear`
- `examples/fundamentals/sim-ml/persistence/train-then-replay` (uses `Cem`)
- `sim/L0/thermostat/tests/d1c_cem_training.rs`, `d1d_reinforce_comparison.rs`, `d2b_stochastic_resonance_baselines.rs`, `d2c_cem_training.rs`

`sim-rl`'s re-exports are designed so these sites can import purely from
`sim_rl::` without `sim_ml_chassis::` showing up unless they want it.

### Dependencies

- `sim-ml-chassis` depends on `sim-core`, `sim-mjcf`, and standard utility
  crates (nalgebra, rand, serde, serde_json, thiserror). Currently
  `sim-ml-bridge` has exactly these plus optional `bevy_ecs` — carry the
  `bevy` feature over unchanged.
- `sim-rl` depends on `sim-ml-chassis`.
- `sim-opt` depends on `sim-ml-chassis` and `sim-thermostat`.
- `sim-opt` depends on `sim-rl` as a **dev-dependency only**, for baseline
  comparisons in competition tests. Not a runtime dep.
- `sim-thermostat` does **not** depend on any algorithm crate.
- Neither `sim-rl` nor `sim-opt` depend on each other at runtime — true peers.

### Trait fit — closed

Walked `sim/L0/ml-bridge/src/algorithm.rs:77-120`. The `Algorithm` trait
takes `&mut VecEnv`, a `TrainingBudget`, a seed, and an `on_epoch`
callback, and returns `Vec<EpochMetrics>`. Fit for each candidate:

- **Simulated Annealing.** Fits `Algorithm::train()` as-is. Each epoch
  evaluates the current candidate solution (run N episodes from fresh
  resets, score by mean return), accepts/rejects via the Metropolis
  criterion at the current temperature, decays the schedule. The inner
  loop uses `VecEnv::step()` the same way CEM and PPO do.
- **Parallel Tempering.** Fits `Algorithm::train()` as-is. Maintain R
  replicas at R temperatures; each epoch advances every replica one
  proposal and runs swap attempts between adjacent temperatures. Report
  the best-performing replica in `policy_artifact()`. Slightly more
  bookkeeping than SA but no trait change.
- **CMA-ES.** Fits `Algorithm::train()` as-is. Each epoch samples a
  population from the current Gaussian, evaluates, updates mean and
  covariance. Structurally identical to CEM with a smarter update.
- **Pure Gibbs-on-Ising.** Does **not** fit `Algorithm::train()` because
  it has no environment. It lives in `sim-thermostat` as a standalone
  `GibbsSampler`, called directly from competition-style tests that
  compare sampler accuracy against Langevin. Those tests are not
  `Competition` runs; they are direct comparisons.

**Decision:** keep exactly one trait (`Algorithm`) in `sim-ml-chassis`. Do
not introduce `Sampler` / `Optimizer` sibling traits. SA, PT, CMA-ES all
implement `Algorithm`. Pure samplers remain standalone in `sim-thermostat`.

## The competition framing

Every optimization / search / learning problem in CortenForge uses the
same experimental recipe. The example below is aspirational — it describes
the shape after `stochastic_resonance()` has been packaged as a
`TaskConfig` (see execution plan step 9):

```rust
use sim_ml_chassis::{Competition, TaskConfig, TrainingBudget};
use sim_ml_chassis::algorithm::Algorithm;
use sim_rl::builders::{build_cem_linear, build_ppo_mlp, build_sac_autograd};
use sim_opt::builders::{build_simulated_annealing, build_parallel_tempering, build_cma_es};
use sim_thermostat::tasks::stochastic_resonance;

let tasks = [stochastic_resonance()];
let comp = Competition::new(/*n_envs=*/ 32, TrainingBudget::Epochs(100), /*seed=*/ 42);

let builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>] = &[
    &build_cem_linear,
    &build_ppo_mlp,
    &build_sac_autograd,
    &build_simulated_annealing,
    &build_parallel_tempering,
    &build_cma_es,
];

let result = comp.run(&tasks, builders).unwrap();
result.print_ranked("stochastic-resonance", "D2c rematch");
```

Three non-obvious facts about this snippet:

1. `Competition::new` takes `(n_envs, budget, seed)` and `run` takes
   `(&[TaskConfig], &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>])` —
   verified against `sim/L0/ml-bridge/src/competition.rs:290,321-325`.
2. `stochastic_resonance()` does not exist today. D2c builds the SR env
   manually (see `d2c_cem_training.rs:86-120`). Wrapping SR as a
   `TaskConfig` is a prerequisite of the rematch and is explicitly listed
   in the execution plan.
3. The `build_*` factories do not exist today either — they are thin
   constructors that each algorithm crate will expose alongside its
   algorithm struct. Creating them is part of the rematch step.

Every comparison is a headless scientific test. Output is a finding: "On
problem class X, algorithm Y beats algorithm Z by factor W." Over time
these accumulate into a heuristic table — *use Gibbs for sampling
equilibrium distributions, use CMA-ES for continuous high-dimensional
design spaces, use parallel tempering for multimodal landscapes, use PPO
when you need a learned state-dependent policy*, and so on.

## What each crate IS and ISN'T

**`sim-ml-chassis`:**

- **IS:** The algorithm chassis. Traits, primitives, networks, Competition,
  artifacts, errors, task configs. Algorithm-neutral. Depends on
  `sim-core`, `sim-mjcf`, and standard utility crates.
- **ISN'T:** An algorithm crate. It does not contain CEM, PPO, SA, or
  anything implementing the `Algorithm` trait concretely (except mock
  algorithms in its own unit tests).

**`sim-rl`:**

- **IS:** A baseline repository of generic RL algorithms — CEM, REINFORCE,
  PPO, TD3, SAC. The control group for every experiment. Re-exports
  common chassis types so downstream consumers can stay in one namespace.
- **ISN'T:** The primary ML layer of CortenForge. Not the destination for
  custom work. Not deprecated — it's the baseline every physics-aware
  claim has to beat.

**`sim-opt`:**

- **IS:** The physics-aware algorithm family. SA, PT, CMA-ES, MH, DE. Each
  algorithm is a thin wrapper over physics primitives from
  `sim-thermostat`, injecting domain knowledge through the update rule.
- **ISN'T:** A replacement for `sim-rl`. It's a peer. Together they cover
  the full "search for good parameters in a simulated environment" space,
  and the competition tests pit them against each other on every problem.

## Decided (not open anymore)

- **Crate names.** `sim-ml-chassis`, `sim-rl`, `sim-opt`. `sim-ml-bridge`
  is split between the first two.
- **Split shape.** Three crates. `sim-rl` and `sim-opt` are siblings, both
  depending on `sim-ml-chassis`.
- **Dependency direction.** `sim-rl → sim-ml-chassis`;
  `sim-opt → sim-ml-chassis`, `sim-opt → sim-thermostat`; `sim-opt → sim-rl`
  (dev-dep only, for baseline comparisons). `sim-thermostat` and
  `sim-ml-chassis` do not depend on `sim-opt`.
- **Naming rationale.** "Chassis" matches the R34 GT-R architecture
  principle — overbuild the chassis, bolt on aftermarket.
- **Where Gibbs lives.** Stays in `sim-thermostat` as a concrete Ising
  instance. General MH wrapper lives in `sim-opt`.
- **Trait count.** One. `Algorithm`. No sibling `Sampler`/`Optimizer` trait.
  SA/PT/CMA-ES implement `Algorithm`; pure samplers are standalone.
- **`task.rs` location.** Chassis (both primitives and stock factories).
- **First algorithm.** Simulated Annealing — simplest member of the family,
  shakes out the trait fit before the more structurally interesting PT.
- **First competition.** D2c rematch: SR + SA + PT versus the linear-RL
  baselines from the original D2c run.
- **Rematch framing.** Physics-aware vs matched-complexity (linear) RL.
  MLP/autograd RL rematch is a deliberate follow-up, not the first test.
- **External-crate dependency.** Rejected (argmin / cmaes). See
  §"Why not depend on argmin / cmaes".

## Open questions (intentional, deferred)

1. **Visual strategy per algorithm.** Parallel Tempering is visually
   compelling (N parallel chains at different temperatures, state swaps
   visible) and deserves a Bevy example. Simulated Annealing is less so
   and can be headless-only initially. Decide per algorithm as each one
   lands; do not block the first rematch on visuals.
2. **sim-ml renovation Phase 5-8 relationship.** The visual examples ladder
   (6-DOF, obstacle, persistence, competition) was in progress before the
   pivot. Post-split: each example lands in either `sim-ml-chassis` (for
   infrastructure demos) or `sim-rl` (for algorithm demos), based on the
   classification in §"Import sites and example classification". The
   competition phase (original issue #25) **must** include `sim-opt`
   algorithms from day one — retire the single-family version of that spec.
3. **MLP-baseline rematch.** After the linear-baseline rematch lands,
   decide whether to also run the RL baselines with MLP policies. Not
   before.

## Execution plan (next session)

Ship as a single PR off `feature/sim-ml-renovation` with a clean sequence
of commits — not a single squashed commit, per the "commit often"
preference. Each numbered step is one commit unless noted.

1. **Scaffold `sim/L0/ml-chassis/`.** New crate, empty `lib.rs`, workspace
   registration. Carry the `bevy` optional feature from `sim-ml-bridge`.
   Compiles green.
2. **Move primitives from `sim-ml-bridge` to `sim-ml-chassis`.** All files
   listed in §"The split". Update intra-crate import paths. Promote
   `best_tracker` from `pub(crate)` to `pub`. Chassis compiles and its
   own tests (`competition.rs` unit tests, `tests/competition.rs`,
   `tests/custom_task.rs`) pass.
3. **Rename remaining `sim-ml-bridge` → `sim-rl`.**
   - Move directory: `sim/L0/ml-bridge/` → `sim/L0/rl/`.
   - Rename package in its `Cargo.toml` (`sim-ml-bridge` → `sim-rl`).
   - Add `sim-ml-chassis` as a dependency.
   - Remaining contents: `cem.rs`, `reinforce.rs`, `ppo.rs`, `td3.rs`,
     `sac.rs`, plus a new `builders.rs` stubbing `build_cem_linear` etc.
     as they get needed.
   - `lib.rs` re-exports: `Algorithm`, `VecEnv`, `Policy`,
     `DifferentiablePolicy`, `StochasticPolicy`, `Tensor`, `TrainingBudget`,
     `EpochMetrics`.
4. **Update all consumers.** Find-and-replace imports across the tree
   using the classification in §"Import sites and example classification":
   - Chassis-only sites: `sim_ml_bridge::` → `sim_ml_chassis::`.
   - Algorithm sites: `sim_ml_bridge::` → `sim_rl::` (re-exports cover the
     chassis types they also reference).
   - Hits: `sim-thermostat` tests (d1c, d1d, d2b, d2c), the 17 example
     crates listed above, docs, and memory files.
5. **Run chassis + sim-rl tests.** Single commit with the clean diff.
   Confirm `cargo xtask grade sim-ml-chassis` and
   `cargo xtask grade sim-rl` both report A — address any dropped
   criteria before proceeding.
6. **Scaffold `sim/L0/opt/`.** New crate, `sim-ml-chassis` and
   `sim-thermostat` as deps, `sim-rl` as dev-dep, empty `lib.rs`. Compiles
   green. Grade A on an empty crate is cheap at this point.
7. **Implement Simulated Annealing in `sim-opt`.** `annealing.rs` +
   `builders::build_simulated_annealing`. Unit tests for the cooling
   schedule and acceptance criterion. Implements `Algorithm`.
8. **Grade `sim-opt`.** Must hit A before the rematch step.
9. **Wrap stochastic resonance as a `TaskConfig` in `sim-thermostat`.**
   Add `sim_thermostat::tasks::stochastic_resonance()` that returns the
   same env currently built inline in `d2c_cem_training.rs:86-120`. This
   unlocks `Competition::run` on SR. Add a regression test that the
   packaged task reproduces the manual build.
10. **Add `build_cem_linear`, `build_ppo_mlp`, `build_sac_autograd`
    builders in `sim-rl::builders`.** Pulled straight from the constructor
    patterns in `d2c_cem_training.rs`. Minimal wrappers — they exist to
    plug into `Competition`.
11. **D2c rematch.** New integration test in `sim/L0/opt/tests/` (or under
    `sim-thermostat/tests/` if it should live alongside D2c). Runs the
    `Competition` snippet from §"The competition framing" with SA added to
    the pool. Report the ranked result.
12. **Decide the next step based on the outcome.** If SA wins, scope PT
    next. If SA loses, debug whether it's trait-fit, schedule tuning, or
    fundamental, before committing to the rest of the family.

## Session sequence

1. **Review session (this one).** Read, nit-pick, fact-check, find gaps,
   close decisions. No code changes. Output: this document, revised.
2. **Execution session.** Follow §"Execution plan" exactly. Success
   criterion: by end of session, `sim-ml-bridge` is gone;
   `sim-ml-chassis` + `sim-rl` + `sim-opt` are in place at A-grade;
   Simulated Annealing is implemented in `sim-opt`; stochastic resonance
   is a `TaskConfig` in `sim-thermostat`; the D2c rematch tells us whether
   SA beats the linear-RL baselines.

## Memory anchors

- `project_thermo_rl_loop_vision.md` — the plaque / north star.
- `feedback_agility_test.md` — the principle that the platform must serve
  both generic and custom without forking.
- `feedback_r34_architecture.md` — the source of the "chassis" naming.
- `project_d2_sr_findings.md` — full D2c results table and the
  "linear-Q is the bottleneck" finding.

This document is the **concrete plan** that those memories point to.
