# ThermCircuitEnv Spec

Status: Active
Created: 2026-04-15
Crate: `sim-therm-env` at `sim/L0/therm-env/`

## 1. What It Is

A typed builder that produces configured `SimEnv` / `VecEnv` instances
from a thermodynamic circuit description. All 7 biological navigation
experiments (see `docs/thermo_computing/biological-navigation-spectrum.md`)
use this builder, just configured with different passive components and
reward functions.

`ThermCircuitEnv` is a struct that implements `Environment` by composing
`SimEnv` internally + domain-specific accessors.

### Value over raw SimEnv

The D2c test (`sim/L0/thermostat/tests/d2c_cem_training.rs:93-127`) has
~35 lines of boilerplate per environment: build MJCF, construct
PassiveStack, install, build obs/act spaces, hand off to SimEnv/VecEnv.
All 7 experiments repeat this pattern. ThermCircuitEnv encapsulates it:

1. Programmatic MJCF generation (N particles, zero-gain actuators)
2. Automatic thermostat construction (gamma sizing, ctrl wiring)
3. Automatic obs/act space construction
4. Domain-specific accessors (`effective_temperature()`, `n_particles()`)
5. One builder -> two output types: `ThermCircuitEnv` (eval) or `VecEnv` (training)

## 2. Where It Lives

New crate: `sim-therm-env` at `sim/L0/therm-env/`.

```
sim/L0/therm-env/
  Cargo.toml
  src/
    lib.rs          (re-exports)
    builder.rs      (ThermCircuitEnvBuilder, MJCF generation)
    env.rs          (ThermCircuitEnv, Environment impl)
    error.rs        (ThermCircuitError)
  tests/
    experiment_4.rs (PN guidance validation)
```

Dependencies:
- sim-core (Model, Data, DVector)
- sim-mjcf (MJCF parsing)
- sim-thermostat (PassiveComponent, LangevinThermostat, landscape components)
- sim-ml-chassis (SimEnv, VecEnv, Environment, ObservationSpace, ActionSpace, Tensor)

Dev dependencies:
- sim-rl (CEM, LinearPolicy for Experiment 4 test)

## 3. API

### 3.1 Builder

```rust
pub struct ThermCircuitEnvBuilder {
    n_particles: usize,
    timestep: f64,                  // default: 0.001
    gamma: f64,                     // default: 0.1
    k_b_t: f64,                    // default: 1.0
    seed: u64,                     // default: 0
    landscape: Vec<Arc<dyn PassiveComponent>>,
    ctrl_temperature: bool,        // default: false
    sub_steps: usize,              // default: 1
    episode_steps: usize,          // default: 1000
    reward_fn: Option<Box<dyn Fn(&Model, &Data) -> f64 + Send + Sync>>,
    done_fn: Option<Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    truncated_fn: Option<Box<dyn Fn(&Model, &Data) -> bool + Send + Sync>>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data) + Send + Sync>>,
}

impl ThermCircuitEnv {
    pub fn builder(n_particles: usize) -> ThermCircuitEnvBuilder;
}

impl ThermCircuitEnvBuilder {
    // ── Physics (all have defaults) ──────────────────────────────
    pub fn timestep(self, h: f64) -> Self;
    pub fn gamma(self, gamma: f64) -> Self;
    pub fn k_b_t(self, k_b_t: f64) -> Self;
    pub fn seed(self, seed: u64) -> Self;

    // ── Energy landscape ─────────────────────────────────────────
    pub fn with(self, c: impl PassiveComponent) -> Self;

    // ── Ctrl channels ────────────────────────────────────────────
    pub fn with_ctrl_temperature(self) -> Self;  // allocates ctrl[0]

    // ── RL config (reward required, rest have defaults) ──────────
    pub fn sub_steps(self, n: usize) -> Self;
    pub fn episode_steps(self, n: usize) -> Self;
    pub fn reward(self, f: impl Fn(&Model, &Data) -> f64 + Send + Sync + 'static) -> Self;
    pub fn done(self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self;
    pub fn truncated(self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self;
    pub fn on_reset(self, f: impl FnMut(&Model, &mut Data) + Send + Sync + 'static) -> Self;

    // ── Build ────────────────────────────────────────────────────
    pub fn build(self) -> Result<ThermCircuitEnv, ThermCircuitError>;
    pub fn build_vec(self, n_envs: usize) -> Result<VecEnv, ThermCircuitError>;
}
```

### 3.2 ThermCircuitEnv

```rust
pub struct ThermCircuitEnv {
    inner: SimEnv,
    n_particles: usize,
    k_b_t: f64,
    ctrl_temperature_idx: Option<usize>,
}

impl Environment for ThermCircuitEnv {
    // All methods delegate to self.inner
}

impl ThermCircuitEnv {
    pub fn n_particles(&self) -> usize;
    pub fn config_k_b_t(&self) -> f64;
    pub fn effective_temperature(&self) -> f64;
    pub fn inner(&self) -> &SimEnv;
    pub fn inner_mut(&mut self) -> &mut SimEnv;
}
```

### 3.3 Error type

```rust
#[derive(Debug, thiserror::Error)]
pub enum ThermCircuitError {
    #[error("missing required field: {field}")]
    MissingField { field: &'static str },

    #[error("n_particles must be >= 1")]
    ZeroParticles,

    #[error(transparent)]
    Env(#[from] EnvError),

    #[error(transparent)]
    Mjcf(#[from] sim_mjcf::MjcfError),
}
```

### 3.4 Defaults

| Field          | Default                                          |
|----------------|--------------------------------------------------|
| timestep       | 0.001                                            |
| gamma          | 0.1                                              |
| k_b_t          | 1.0                                              |
| seed           | 0                                                |
| sub_steps      | 1                                                |
| episode_steps  | 1000                                             |
| done           | `\|_m, _d\| false`                               |
| truncated      | `\|_m, d\| d.time > episode_steps * sub_steps * timestep` |
| reward         | **required** (build fails without it)            |

## 4. MJCF Generation

The builder produces a minimal MJCF string at build time. For N
particles with M ctrl channels:

```xml
<mujoco model="therm-circuit-{N}">
  <option timestep="{h}" gravity="0 0 0" integrator="Euler">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="p0">
      <joint name="x0" type="slide" axis="1 0 0" damping="0"/>
      <geom type="sphere" size="0.05" mass="1"/>
    </body>
    <!-- ... pN-1 -->
  </worldbody>
  <actuator>
    <general name="ctrl_0" joint="x0" gainprm="0" biasprm="0 0 0"
             ctrllimited="true" ctrlrange="0 10"/>
    <!-- ... ctrl_M-1 -->
  </actuator>
</mujoco>
```

Design notes:
- Slide joints: 1 DOF each, nq = nv = N. PassiveComponent dof indices
  map 1:1 to particle indices.
- Zero gravity, Euler integrator, contact disabled: pure Langevin dynamics.
- Joint damping = 0: damping comes from the thermostat PassiveComponent.
- Zero-gain actuators (`gainprm="0" biasprm="0 0 0"`): produce zero
  force, exist only to give `data.ctrl` the right length. Joint target
  is irrelevant but required by MJCF syntax.
- Mass = 1: simplifies FDT relation. Configurable later if needed.

## 5. Internal Wiring

### 5.1 build() flow

1. Generate MJCF XML from `n_particles` + ctrl layout.
2. Parse XML via `sim_mjcf::load_model()` -> `Model`.
3. Construct `LangevinThermostat::new(gamma_vec, k_b_t, seed, traj_id=0)`.
   If `ctrl_temperature`, chain `.with_ctrl_temperature(0)`.
4. Build `PassiveStack::builder().with(thermostat).with(c1).with(c2)...build()`.
5. `stack.install(&mut model)`.
6. `Arc::new(model)`.
7. Build `ObservationSpace::builder().all_qpos().all_qvel().build(&model)`.
8. Build `ActionSpace::builder().all_ctrl().build(&model)`.
9. Build `SimEnv::builder(model).observation_space(...).action_space(...)
   .reward(...).done(...).truncated(...).sub_steps(...).build()`.
10. Return `ThermCircuitEnv { inner: sim_env, ... }`.

### 5.2 build_vec() flow

Same as build() steps 1-8, then:
9. Build `VecEnv::builder(model, n_envs)...build()`.
10. Return `VecEnv`.

### 5.3 Thermostat ownership

The builder constructs the thermostat internally. The user specifies
`gamma` and `k_b_t`; the builder creates `LangevinThermostat` with
`gamma_vec = DVector::from_element(n_particles, gamma)` and wires
`with_ctrl_temperature()` if requested. The user never touches
`LangevinThermostat` directly.

Landscape components (DoubleWellPotential, PairwiseCoupling,
OscillatingField, ExternalField, RatchetPotential) are added via
`.with()` — the user constructs them and the builder wraps them in Arc.

## 6. Per-Phase Iteration Loop

Each phase = one session. The session follows this loop:

**Step 0: Recon** — Cold start. Read this spec (current phase section +
implementation log of prior phases). Read the source files the phase
touches. Pure information gathering, no decisions yet.

**Step 1: Pre-flight** — Analyze what recon found. Do the spec's
assumptions still hold against current code? What are the 2-3 likeliest
failure modes? Does anything need adjusting before code starts? This is
the "sharpen the axe" step — targeted verification, not exhaustive.

**Step 2: Implement** — Write code + tests for the phase deliverable.

**Step 3: Verify** — Run the tests. Run clippy. Check that the crate
builds in workspace context (`cargo build -p sim-therm-env`). For phases
2+, run the full crate test suite to catch regressions from earlier phases.

**Step 4: Review** — Read back the code just written. Does it match the
spec? Any deviations? Any surprises or discoveries?

**Step 5: Update spec + memory** — Write the implementation log entry
for this phase in section 8 (three categories: **Deviations** from spec,
**Discoveries** that inform later phases, **Confirmed** assumptions).
Update memory files if anything learned affects future sessions.
Check whether discoveries require changes to later phases' plans.

**Step 6: Next-session prompt** — Write a self-contained cold-start
prompt for the next phase. Everything the next session needs to orient
without history: what was built, what changed from spec, what to read,
what to build, operating discipline. User pastes it, new session picks
up clean.

## 7. Phase Plans

### Phase 1: Crate skeleton + MJCF generation

Deliverable: `generate_mjcf(n_particles, n_ctrl, timestep) -> String`
in a new `sim-therm-env` crate with Cargo.toml wired into the workspace.

Tests:
- Generated MJCF parses via sim_mjcf::load_model
- Correct nq, nv, nu for 1, 2, 8 particles
- Correct joint/actuator naming
- Zero-gain actuators produce zero force
- Workspace builds clean (`cargo build -p sim-therm-env`)

### Phase 2: Builder -> ThermCircuitEnv (single env)

Deliverable: `ThermCircuitEnv::builder(n)...build()` producing a working
`ThermCircuitEnv` that implements `Environment`.

Tests:
- Build 1-particle env, reset returns obs of correct dim
- Step with zero action advances time
- Reward closure fires correctly
- Truncation triggers at episode_steps
- Double-well forces are applied (particle moves toward well)
- Ctrl-temperature modulates noise (deterministic damping check)
- Domain accessors: n_particles, config_k_b_t, effective_temperature
- Builder validation: missing reward -> error, zero particles -> error
- Multiple landscape components stack correctly

### Phase 3: Builder -> VecEnv (batched)

Deliverable: `ThermCircuitEnv::builder(n)...build_vec(n_envs)` producing
a working `VecEnv`.

Tests:
- Build 4-env VecEnv, step all, reset all
- Observations have correct batch shape [n_envs, obs_dim]
- Reward computed independently per env
- Truncation + auto-reset works per env

### Phase 4: Experiment 4 validation (PN guidance)

Deliverable: Integration test validating the full thermo-RL loop.

Setup:
- 1-particle circuit, double well (DeltaV=3, x0=1), ctrl-temperature
- gamma=10, kT=1, timestep=0.001, sub_steps=100, episode_steps=1000
- Reward: `-(qpos[0] - target)^2` where target = +x0

Test:
- CEM with LinearPolicy(2, 1) trains for 50 epochs on 16-env VecEnv
- Gate: final mean_reward > constant-temperature baseline
- `#[ignore]` (release-mode, ~1-2 min)

## 8. Explicit Deferrals

| Item | Reason | When |
|------|--------|------|
| Per-env thermostat seeding in VecEnv | Requires VecEnv/BatchSim changes | Post-validation |
| `potential_energy()` accessor | Needs PassiveComponent trait extension or typed refs | Post-validation |
| Multi-axis particles (2D/3D) | Ball/free joints change qpos/qvel indexing | When Experiment 5 or 6 needs it |
| Custom obs/act spaces | Current default (all_qpos+all_qvel / all_ctrl) sufficient | When an experiment needs it |
| Experiments 1-7 | This spec validates with Experiment 4 only | Sequential after Phase 4 |
| Per-particle gamma | All particles share one gamma value | When coupling experiments need it |
| Per-particle mass | All particles have mass=1 | When needed |

## 9. Implementation Log

### Phase 1

Status: Complete (2026-04-15)

**Deviations:** None.

**Discoveries:**
- Model joint/actuator names are `jnt_name: Vec<Option<String>>` and
  `actuator_name: Vec<Option<String>>` — not a typed collection with
  `.name` accessor. Phase 2 builder should use `model.joint_id(name)`
  for lookups.
- `write!` to String triggers `let_underscore_must_use` (workspace deny).
  Pattern: use `write!(xml, ...).ok();` (bare statement, no `let _ =`).

**Confirmed:**
- Slide joints: nq = nv = 1 per joint, N particles -> nq = nv = N.
- Zero-gain actuator (`gainprm="0" biasprm="0 0 0"`) produces zero
  `qfrc_actuator` regardless of `ctrl` value.
- `data.ctrl` length = `model.nu` = number of actuators.
- `sim_mjcf::load_model(&str) -> Result<Model>` parses inline XML.

**Next-session prompt (Phase 2):**
(see below)

### Phase 2

Status: Complete (2026-04-15)

**Deviations:**
- Error type gained `Space(#[from] SpaceError)` variant (spec only had
  `MissingField`, `ZeroParticles`, `Env`, `Mjcf`). Needed for `?` on
  obs/act space construction.
- `PassiveStackBuilder` gained `with_arc(Arc<dyn PassiveComponent>)`
  method in sim-thermostat (4-line addition). Needed because the
  ThermCircuitEnvBuilder stores landscape components as
  `Vec<Arc<dyn PassiveComponent>>` and the existing `with()` takes
  `impl PassiveComponent` (would double-wrap).
- Builder stores landscape as `Vec<Arc<dyn PassiveComponent>>` rather
  than `Vec<Arc<dyn PassiveComponent>>` with a separate `Arc` wrapper —
  same semantic, slightly different internal type.
- Added manual `Debug` impl for `ThermCircuitEnv` (SimEnv has one;
  needed for `unwrap_err()` in tests).

**Discoveries:**
- `SimEnvBuilder` requires ALL of `obs_space`, `act_space`, `reward`,
  `done`, AND `truncated` — all five are mandatory. The
  ThermCircuitEnvBuilder must supply defaults for `done` (always false)
  and `truncated` (time-based) before passing to SimEnvBuilder.
- `ObservationSpace::builder().all_qpos().all_qvel().build()` returns
  `Result<_, SpaceError>`, not `EnvError`. Need separate `#[from]` in
  the error enum.
- `Box<dyn Fn(...)>` passed to `SimEnvBuilder::reward(impl Fn)` causes
  double-boxing. Negligible overhead, but worth noting for Phase 3 if
  VecEnv builder has the same shape.
- No ctrl channels → `nu = 0` → `ActionSpace::all_ctrl()` produces
  dim-0 action space → tests pass `Tensor::zeros(&[0])` for actions.

**Confirmed:**
- `PassiveStack::builder().with(thermostat).with(well).build()` +
  `stack.install(&mut model)` works exactly as in the D2c reference.
- `LangevinThermostat::new(gamma_vec, k_b_t, seed, 0)
  .with_ctrl_temperature(0)` wires ctrl[0] as temperature multiplier.
- Damping at gamma=10 over 100 sub-steps at h=0.001 produces visible
  velocity decay (v ≈ 0.37 from exp(-1)), confirming thermostat is
  active in the composed env.
- Double-well restoring force at x=0.5 pushes particle toward x=+1.0
  well, confirming landscape components are wired through cb_passive.
- Default truncation `d.time > max_time` fires at the right step count.

**Next-session prompt (Phase 3):**
(written to terminal output at end of Phase 2 session)

### Phase 3

Status: Complete (2026-04-15)

**Deviations:**
- Extracted shared steps 1-8 from `build()` into a private
  `prepare() -> Result<PreparedCircuit, ThermCircuitError>` method.
  Both `build()` and `build_vec()` call it, eliminating ~50 lines of
  duplication. `PreparedCircuit` is a private struct at module scope.
- `build()` no longer clones the `Arc<Model>` — passes it directly to
  `SimEnv::builder()` since it's not used afterward. Functionally identical.

**Discoveries:**
- `VecEnvBuilder::on_reset` takes 3 args `(&Model, &mut Data, usize)` —
  the extra `env_index` parameter is not exposed through the
  ThermCircuitEnvBuilder's 2-arg `on_reset`. Wrapped with
  `move |m, d, _idx| on_reset(m, d)`. Per-env seeding via index is
  already listed as deferred in §8.
- VecEnvBuilder stores closures as `Arc` (vs SimEnvBuilder's `Box`),
  so passing `Box<dyn Fn>` to its methods produces `Arc<Box<dyn Fn>>`.
  Same double-indirection as Phase 2, negligible overhead.

**Confirmed:**
- VecEnvBuilder requires ALL 5 closures, same as SimEnvBuilder.
- `ThermCircuitError`'s `#[from] EnvError` handles `?` from
  `VecEnvBuilder::build()` without any error type changes.
- All 26 Phase 1-2 tests pass unchanged with the refactored `build()`
  path (no regressions from `prepare()` extraction).
- 5 new Phase 3 tests all pass: step+reset, batch shape (1p and 2p),
  per-env reward, truncation + auto-reset.

**Next-session prompt (Phase 4):**
(written below)

### Phase 4

Status: Not started

**Deviations:** (updated after implementation)
**Discoveries:** (updated after implementation)
**Confirmed:** (updated after implementation)

**Next-session prompt (post-ThermCircuitEnv):**
(written at end of Phase 4 session)
