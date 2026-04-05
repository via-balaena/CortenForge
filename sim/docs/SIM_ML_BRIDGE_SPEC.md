# sim-ml-bridge Spec

> **Status**: Draft v3  
> **Crate**: `sim-ml-bridge` (Layer 0 — zero Bevy, zero ML framework deps)  
> **Location**: `sim/L0/ml-bridge/`  
> **Depends on**: `sim-core`, `sim-types`  
> **Depended on by**: nothing in-tree (consumer crate)

## 1. Purpose

A clean boundary layer that lets RL training loops consume the simulator
without the simulator knowing anything about ML. The sim stays completely
pure — no training, policy, gradient, or framework concepts leak into
sim-core or sim-types.

This crate is **the foundation** for CortenForge's own lean autodiff and RL
library, built from scratch. No Burn, no tch-rs, no external ML framework.

## 2. Design Principles

1. **Sim purity** — sim-core and sim-types gain zero new dependencies or
   traits. All ML-facing abstractions live in sim-ml-bridge.
2. **Composition over inheritance** — the bridge wraps `Model` + `Data` (and
   `BatchSim`); it doesn't subclass or modify them.
3. **Configurable observation/action spaces** — RL tasks vary wildly in what
   they observe and control. The bridge provides builders that select which
   slices of `Data` map to observation/action tensors.
4. **Reward and termination are user-defined** — these are task-specific.
   The bridge provides the hook; the user provides the logic.
5. **f64 → f32 is a conscious boundary** — the sim is `f64` throughout.
   ML is `f32`. The bridge owns this conversion explicitly.
6. **Vectorized environments from day one** — the bridge composes with
   `BatchSim` for parallel environment stepping. Single-env is just
   `n = 1`.
7. **Batched tensors, not vectors of tensors** — VecEnv produces a single
   `Tensor` of shape `[n_envs, obs_dim]`, not `Vec<Tensor>`. One allocation,
   one policy forward pass, one memory layout. Same for actions in.

## 3. Foundation Audit

Before building on top of sim-core, we audited the foundation:

| Component | Verdict | Notes |
|-----------|---------|-------|
| `Data` struct | **Solid** | Monolithic flat arrays = correct final form (MuJoCo pattern). Cache-coherent, SIMD-ready, batch-friendly. Only `qpos`/`qvel` are state; everything else derived. Zero unsafe. Staleness guard test on struct size. |
| `Data::reset()` | **Solid** | Fully deterministic. No RNG state. Resets all fields including `qfrc_applied`, `xfrc_applied`, `ctrl`. Staleness guard catches missing fields. |
| `Model` | **Solid** | Immutable post-construction. Rich name→id lookups. `make_data()` pre-allocates everything — zero heap allocation during stepping. |
| `BatchSim` | **Solid** | Bit-exact determinism (proven). Per-env isolation (proven). Flexible reset (single, masked, all). rayon parallel stepping. 24 unit + 4 integration tests + 3 examples. |
| `step()` / `step1()` / `step2()` | **Solid** | Clean Result-based API. Split-step supports force injection between phases. RK4 and Euler paths. |
| Data accessors | **Solid** | `joint_qpos()`, `joint_qvel()`, `sensor_data()`, `sensor_scalar()`, `contacts_involving_geom()` — stable, indexed by model metadata. |

**Conclusion**: no structural refactoring needed. Build directly on top.

## 4. Architecture

```
sim-ml-bridge/
├── Cargo.toml
├── src/
│   ├── lib.rs          — Public API re-exports
│   ├── tensor.rs       — Tensor, TensorSpec
│   ├── space.rs        — ObservationSpace, ActionSpace, builders
│   ├── env.rs          — Environment trait + SimEnv
│   ├── vec_env.rs      — VecEnv (BatchSim composition)
│   └── error.rs        — SpaceError, EnvError
```

### Dependency graph

```
sim-types ← sim-core ← sim-ml-bridge
                ↑              ↑
             (no new deps)  (no ML framework deps)
```

sim-ml-bridge depends on sim-core (and transitively sim-types).
Nothing in the sim crate graph depends on sim-ml-bridge.

## 5. Core Types

### 5.1 Tensor

```rust
/// A flat f32 buffer with shape metadata.
///
/// No autograd, no device placement, no strides. This is the simplest
/// possible tensor — a typed Vec<f32> that knows its shape. We will
/// grow it later (autodiff, GPU buffers) but the interface stays stable.
#[derive(Debug, Clone, PartialEq)]
pub struct Tensor {
    data: Vec<f32>,
    shape: Vec<usize>,
}
```

**Key decisions**:
- `f32`, not `f64` — ML convention. The bridge owns the f64→f32 conversion.
- Shape is dynamic (`Vec<usize>`) — observation/action dimensions vary per task.
- No strides, no views, no autograd. Growth path is clear but not premature.

**Core API**:
```rust
impl Tensor {
    pub fn zeros(shape: &[usize]) -> Self;
    pub fn from_slice(data: &[f32], shape: &[usize]) -> Self;
    pub fn from_f64_slice(data: &[f64], shape: &[usize]) -> Self;  // f64→f32
    pub fn try_from_slice(data: &[f32], shape: &[usize]) -> Result<Self, TensorError>;
    pub fn try_from_f64_slice(data: &[f64], shape: &[usize]) -> Result<Self, TensorError>;
    pub fn shape(&self) -> &[usize];
    pub fn ndim(&self) -> usize;
    pub fn len(&self) -> usize;           // total elements (product of shape)
    pub fn is_empty(&self) -> bool;
    pub fn as_slice(&self) -> &[f32];
    pub fn as_mut_slice(&mut self) -> &mut [f32];

    /// View a row of a 2D tensor: shape [N, D] → row i is &[f32] of length D.
    /// Panics if ndim != 2 or i >= shape[0].
    pub fn row(&self, i: usize) -> &[f32];
    pub fn row_mut(&mut self, i: usize) -> &mut [f32];
}
```

`row()` / `row_mut()` exist because VecEnv works with batched tensors
of shape `[n_envs, dim]` and needs to scatter actions into individual envs.
No strides, no general indexing — just the one operation we actually need.

**Validation policy**: `from_slice` and `from_f64_slice` **panic** if
`data.len() != product(shape)`. This is always a programming error — a shape
mismatch between what you have and what you declared. Panicking matches
nalgebra's convention and keeps the hot-path API ergonomic (no unwrapping).
`try_from_slice` and `try_from_f64_slice` return `Result<Self, TensorError>`
for callers who need to validate external input (e.g., deserializing a
tensor from disk or network). Internal bridge code (extract, apply) uses
the panicking variants — shapes are validated once at build() time and
guaranteed correct thereafter.

### 5.2 TensorSpec

```rust
/// Describes the shape and bounds of an observation or action tensor.
///
/// Used by environments to declare their observation/action spaces
/// so that policies can be constructed with matching dimensions.
#[derive(Debug, Clone)]
pub struct TensorSpec {
    pub shape: Vec<usize>,
    pub low: Option<Vec<f32>>,     // per-element lower bounds (None = unbounded)
    pub high: Option<Vec<f32>>,    // per-element upper bounds (None = unbounded)
}
```

### 5.3 ObservationSpace

Configurable mapping from `Data` fields to a flat `Tensor`.

```rust
/// Selects which slices of Data become the observation vector.
///
/// The observation is built by concatenating selected slices in the
/// order they were added. The total dimension is the sum of all
/// selected slice lengths.
pub struct ObservationSpace {
    extractors: Vec<Extractor>,  // ordered list of field extractions
    dim: usize,                  // total observation dimension (cached)
}

/// A single extraction: "read these indices from this field of Data."
///
/// Ranges on flat fields (qpos, qvel, sensordata) are element indices.
/// Ranges on per-body fields (xpos, xquat, cvel) are BODY indices —
/// extraction flattens automatically (3 floats per body for xpos,
/// 4 for xquat, 6 for cvel).
enum Extractor {
    // Flat fields — range is element indices into the DVector/Vec
    Qpos(Range<usize>),           // data.qpos[range]
    Qvel(Range<usize>),           // data.qvel[range]
    Qacc(Range<usize>),           // data.qacc[range]
    Ctrl(Range<usize>),           // data.ctrl[range]
    Sensordata(Range<usize>),     // data.sensordata[range]
    ActuatorForce(Range<usize>),  // data.actuator_force[range]
    QfrcConstraint(Range<usize>), // data.qfrc_constraint[range]

    // Per-body fields — range is body indices, flattened on extraction
    Xpos(Range<usize>),           // data.xpos[range] → 3 floats per body
    Xquat(Range<usize>),          // data.xquat[range] → 4 floats per body
    Cvel(Range<usize>),           // data.cvel[range] → 6 floats per body

    // Scalars
    ContactCount,                 // data.ncon as f32 (1 float)
    Time,                         // data.time as f32 (1 float)
    Energy,                       // [kinetic, potential] (2 floats)
}
```

**Builder API**:
```rust
impl ObservationSpace {
    pub fn builder() -> ObservationSpaceBuilder;
    pub fn dim(&self) -> usize;
    pub fn spec(&self) -> TensorSpec;

    /// Extract observation from a single Data instance.
    pub fn extract(&self, data: &Data) -> Tensor;

    /// Extract observations from N Data instances into a batched tensor.
    /// Returns shape [n, dim]. Writes directly into a contiguous buffer —
    /// no intermediate per-env tensors.
    pub fn extract_batch(&self, envs: impl ExactSizeIterator<Item = &Data>) -> Tensor;
}

impl ObservationSpaceBuilder {
    // Flat fields — range is element indices
    pub fn qpos(self, range: Range<usize>) -> Self;
    pub fn qvel(self, range: Range<usize>) -> Self;
    pub fn qacc(self, range: Range<usize>) -> Self;
    pub fn ctrl(self, range: Range<usize>) -> Self;
    pub fn sensordata(self, range: Range<usize>) -> Self;
    pub fn actuator_force(self, range: Range<usize>) -> Self;
    pub fn qfrc_constraint(self, range: Range<usize>) -> Self;

    // Per-body fields — range is body indices
    pub fn xpos(self, body_range: Range<usize>) -> Self;
    pub fn xquat(self, body_range: Range<usize>) -> Self;
    pub fn cvel(self, body_range: Range<usize>) -> Self;

    // By name — resolved at build() time from Model's name→id maps
    pub fn sensor(self, name: &str) -> Self;

    // Scalars
    pub fn contact_count(self) -> Self;
    pub fn time(self) -> Self;
    pub fn energy(self) -> Self;

    // Shorthands — expanded at build() time using Model dimensions
    pub fn all_qpos(self) -> Self;          // 0..nq
    pub fn all_qvel(self) -> Self;          // 0..nv
    pub fn all_sensordata(self) -> Self;    // 0..nsensordata

    /// Validate all ranges against Model and freeze the space.
    pub fn build(self, model: &Model) -> Result<ObservationSpace, SpaceError>;
}
```

The builder validates ranges against the model at `build()` time — out-of-bounds
ranges produce `SpaceError`, not panics at extraction time.

Named sensors (`sensor("touch")`) store the name as a string in the builder
and resolve to `model.sensor_name_to_id` + `model.sensor_adr` + `model.sensor_dim`
at `build()` time. If the name isn't found, `build()` returns `SpaceError`.

### 5.4 ActionSpace

Configurable mapping from a flat `Tensor` to `Data` fields.

```rust
/// Selects which slices of Data the action vector writes to.
pub struct ActionSpace {
    injectors: Vec<Injector>,
    dim: usize,
}

enum Injector {
    Ctrl(Range<usize>),          // action[offset..] → data.ctrl[range]
    QfrcApplied(Range<usize>),   // action[offset..] → data.qfrc_applied[range]
    XfrcApplied(Range<usize>),   // action[offset..] → data.xfrc_applied[range] (6 per body)
    MocapPos(Range<usize>),      // action[offset..] → data.mocap_pos[range] (3 per body)
    MocapQuat(Range<usize>),     // action[offset..] → data.mocap_quat[range] (4 per body)
}
```

**Builder API**:
```rust
impl ActionSpace {
    pub fn builder() -> ActionSpaceBuilder;
    pub fn dim(&self) -> usize;
    pub fn spec(&self, model: &Model) -> TensorSpec;  // includes ctrl limits

    /// Apply action to a single Data instance.
    /// Casts f32→f64. Clamps Ctrl injectors to model.actuator_ctrlrange.
    pub fn apply(&self, action: &Tensor, data: &mut Data);

    /// Apply batched actions [n, dim] to N Data instances.
    pub fn apply_batch(&self, actions: &Tensor, envs: impl ExactSizeIterator<Item = &mut Data>);
}

impl ActionSpaceBuilder {
    pub fn ctrl(self, range: Range<usize>) -> Self;
    pub fn all_ctrl(self) -> Self;                       // 0..nu
    pub fn qfrc_applied(self, range: Range<usize>) -> Self;
    pub fn xfrc_applied(self, body_range: Range<usize>) -> Self;
    pub fn mocap_pos(self, body_range: Range<usize>) -> Self;
    pub fn mocap_quat(self, body_range: Range<usize>) -> Self;
    pub fn build(self, model: &Model) -> Result<ActionSpace, SpaceError>;
}
```

**Clamping behavior**: `apply()` clamps `Ctrl` injector values to
`model.actuator_ctrlrange[i]` when the actuator has `ctrllimited = true`.
Other injectors (`QfrcApplied`, `XfrcApplied`, `MocapPos/Quat`) are written
without clamping — the sim handles out-of-range forces/positions gracefully.
This is standard RL behavior: policies output unbounded Gaussians, the
environment clips to actuator limits.

### 5.5 StepResult

```rust
/// The output of a single environment step.
pub struct StepResult {
    pub observation: Tensor,
    pub reward: f64,       // f64 — reward precision matters for learning
    pub done: bool,        // episode terminated (true terminal state)
    pub truncated: bool,   // episode truncated (time limit, not terminal)
}
```

Uses Gymnasium's `done` / `truncated` distinction — this matters for correct
value bootstrapping in RL. When `done = true`, the value of the terminal
state is zero. When `truncated = true`, the value must be bootstrapped from
the next state (the agent didn't truly fail).

### 5.6 VecStepResult

```rust
/// Batch step result from VecEnv.
///
/// Observations and rewards are batched — one contiguous tensor and one
/// Vec, not a Vec<Tensor>. This avoids N allocations per step and produces
/// the layout policies expect (batch dimension first).
pub struct VecStepResult {
    /// Shape: [n_envs, obs_dim]. Row i is env i's observation.
    /// For envs that were auto-reset (done or truncated), this is the
    /// INITIAL observation of the new episode, not the terminal one.
    pub observations: Tensor,

    /// Rewards from the completed step (before auto-reset).
    pub rewards: Vec<f64>,

    /// True if env i's episode terminated this step (true terminal).
    pub dones: Vec<bool>,

    /// True if env i's episode was truncated this step (time limit).
    pub truncateds: Vec<bool>,

    /// Terminal observations for envs that were auto-reset.
    /// `terminal_observations[i]` is `Some(obs)` if env i was reset this step
    /// (the observation from the terminal state, before reset). `None` otherwise.
    /// Needed for correct value bootstrapping on truncated episodes.
    pub terminal_observations: Vec<Option<Tensor>>,
}
```

**Why `terminal_observations`**: when VecEnv auto-resets a done/truncated
env, the observation in `observations` is the post-reset initial obs (so the
training loop can immediately use it as input to the next policy forward
pass). But the RL algorithm needs the terminal obs for value estimation.
Gymnasium solves this with `info["final_observation"]`. We make it a
first-class field.

## 6. The Environment Trait

```rust
/// A single RL environment wrapping Model + Data.
///
/// Most users should use `SimEnv` (closure-based) rather than implementing
/// this trait directly. The trait exists for cases where closures don't
/// suffice (multi-phase environments, complex state machines, etc.).
pub trait Environment {
    fn observation_space(&self) -> &ObservationSpace;
    fn action_space(&self) -> &ActionSpace;
    fn observe(&self) -> Tensor;
    fn step(&mut self, action: &Tensor) -> Result<StepResult, StepError>;
    fn reset(&mut self) -> Result<Tensor, ResetError>;
    fn model(&self) -> &Model;
    fn data(&self) -> &Data;
}
```

### 6.1 SimEnv — closure-based environment

```rust
/// A general-purpose Environment built from closures.
///
/// Handles observation extraction, action injection, stepping, and reset.
/// The user provides only the task-specific parts: reward, termination,
/// and optional reset customization (domain randomization).
pub struct SimEnv {
    model: Arc<Model>,
    data: Data,
    obs_space: ObservationSpace,
    act_space: ActionSpace,
    reward_fn: Box<dyn Fn(&Model, &Data) -> f64>,
    done_fn: Box<dyn Fn(&Model, &Data) -> bool>,
    truncated_fn: Box<dyn Fn(&Model, &Data) -> bool>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data)>>,
    sub_steps: usize,
}
```

**Builder API**:
```rust
impl SimEnv {
    pub fn builder(model: Arc<Model>) -> SimEnvBuilder;
}

impl SimEnvBuilder {
    pub fn observation_space(self, space: ObservationSpace) -> Self;
    pub fn action_space(self, space: ActionSpace) -> Self;
    pub fn reward(self, f: impl Fn(&Model, &Data) -> f64 + 'static) -> Self;
    pub fn done(self, f: impl Fn(&Model, &Data) -> bool + 'static) -> Self;
    pub fn truncated(self, f: impl Fn(&Model, &Data) -> bool + 'static) -> Self;
    pub fn sub_steps(self, n: usize) -> Self;

    /// Hook called after Data::reset(). Use for domain randomization:
    /// randomize initial joint angles, object positions, etc.
    /// The closure is FnMut so it can own and advance its own RNG.
    pub fn on_reset(self, f: impl FnMut(&Model, &mut Data) + 'static) -> Self;

    pub fn build(self) -> Result<SimEnv, EnvError>;
}
```

**`SimEnv::step()` → `Result<StepResult, StepError>`**:
1. `self.act_space.apply(action, &mut self.data)`
2. For each sub-step `k` in `0..sub_steps`:
   a. `self.data.step(&self.model)?` — propagates physics errors to caller
   b. If `(self.done_fn)(&self.model, &self.data)` → break (early termination)
3. Compute `reward = (self.reward_fn)(&self.model, &self.data)`
4. Compute `done = (self.done_fn)(&self.model, &self.data)`
5. Compute `truncated = (self.truncated_fn)(&self.model, &self.data)`
6. Extract `obs = self.obs_space.extract(&self.data)`
7. Return `Ok(StepResult { observation: obs, reward, done, truncated })`

Unlike VecEnv, SimEnv propagates physics errors directly — the caller
decides whether to reset and retry.

**`SimEnv::reset()` → `Result<Tensor, ResetError>`**:
1. `self.data.reset(&self.model)` — standard Data reset (zeros everything)
2. If `on_reset_fn` is set: `(self.on_reset_fn)(&self.model, &mut self.data)` — domain randomization
3. `self.data.forward(&self.model)?` — recompute derived quantities from modified state
4. Extract and return initial observation

Step 3 is important: if `on_reset` modifies `qpos` (e.g., random initial angles),
the derived quantities (`xpos`, `xquat`, `sensordata`) must be recomputed
before extracting the observation. `forward()` does this without integrating.
If `forward()` fails, the on_reset hook likely put Data into an invalid
configuration (e.g., NaN in qpos) — this is a `ResetError`.

## 7. VecEnv — Vectorized Environments

Composes directly with `BatchSim`.

```rust
/// Vectorized environment: N parallel environments sharing one Model.
///
/// Wraps BatchSim and adds observation/action/reward/done semantics.
/// All environments use identical observation and action spaces
/// (same Model, same task). Individual environments can be at different
/// points in their episodes.
pub struct VecEnv {
    batch: BatchSim,
    obs_space: ObservationSpace,
    act_space: ActionSpace,
    reward_fn: Arc<dyn Fn(&Model, &Data) -> f64 + Send + Sync>,
    done_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    truncated_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    on_reset_fn: Option<Box<dyn FnMut(&Model, &mut Data, usize) + Send>>,
    sub_steps: usize,
}
```

### 7.1 VecEnvBuilder

```rust
impl VecEnv {
    pub fn builder(model: Arc<Model>, n_envs: usize) -> VecEnvBuilder;
}

impl VecEnvBuilder {
    pub fn observation_space(self, space: ObservationSpace) -> Self;
    pub fn action_space(self, space: ActionSpace) -> Self;
    pub fn reward(self, f: impl Fn(&Model, &Data) -> f64 + Send + Sync + 'static) -> Self;
    pub fn done(self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self;
    pub fn truncated(self, f: impl Fn(&Model, &Data) -> bool + Send + Sync + 'static) -> Self;
    pub fn sub_steps(self, n: usize) -> Self;

    /// Hook called after Data::reset() for each env that resets.
    ///
    /// Receives `(model, data, env_index)` — the env index lets the closure
    /// use per-env RNG seeds or deterministic randomization schedules.
    ///
    /// `FnMut + Send`: `FnMut` because resets are sequential (the closure
    /// can own mutable state like an RNG directly). `Send` so VecEnv is
    /// `Send`. See §7.3 for the full threading model.
    pub fn on_reset(self, f: impl FnMut(&Model, &mut Data, usize) + Send + 'static) -> Self;

    pub fn build(self) -> Result<VecEnv, EnvError>;
}
```

### 7.2 Core API

```rust
impl VecEnv {
    /// Number of parallel environments.
    pub fn n_envs(&self) -> usize;

    /// Step all environments.
    ///
    /// `actions` has shape [n_envs, act_dim]. Row i is applied to env i.
    /// All N sub-steps run for all envs (no per-env early exit — §9).
    /// Done/truncated evaluated once at final state, then auto-reset (§7.5).
    /// Returns `Err(VecStepError)` only for bridge errors (§7.4).
    pub fn step(&mut self, actions: &Tensor) -> Result<VecStepResult, VecStepError>;

    /// Reset all environments, returning initial observations.
    /// Shape: [n_envs, obs_dim].
    pub fn reset_all(&mut self) -> Result<Tensor, ResetError>;

    /// Access underlying BatchSim for advanced use.
    pub fn batch(&self) -> &BatchSim;
    pub fn batch_mut(&mut self) -> &mut BatchSim;

    /// Access shared model.
    pub fn model(&self) -> &Model;
}
```

### 7.3 Threading Model

**Physics stepping** is parallelized via `BatchSim::step_all()` (rayon
`par_iter_mut`). The bridge's own work (action injection, reward/done
evaluation, observation extraction, resets) runs sequentially after
`step_all()` returns. This is the right split: physics dominates wall time;
the bridge's per-env overhead is trivial (<1 µs per env).

**Why closures still need `Send + Sync`**: reward, done, and truncated
closures are `Fn + Send + Sync` not because they're called in parallel, but
so that `VecEnv` itself is `Send + Sync` — essential if the training loop
runs on a different thread or uses async.

**`on_reset_fn` is `FnMut + Send`**: `FnMut` because resets are sequential
(one closure invocation at a time) — the closure can own mutable state
directly. `Send` so VecEnv remains `Send`. No `Sync` needed (exclusive
access via `&mut self`).

```rust
let mut rngs: Vec<StdRng> = (0..n_envs)
    .map(|i| StdRng::seed_from_u64(42 + i as u64))
    .collect();

VecEnv::builder(model, n_envs)
    .on_reset(move |_model, data, env_idx| {
        // Each env gets its own deterministic RNG
        data.qpos[0] += rngs[env_idx].gen_range(-0.1..0.1);
    })
    .build()?;
```

No `Arc<Mutex<RNG>>`, no interior mutability. The cost: resets are
sequential. Acceptable because resets are rare (once per episode) and cheap
relative to stepping (just `Data::reset()` + `forward()`).

### 7.4 Error Handling in VecEnv::step()

`BatchSim::step_all()` returns `Vec<Option<StepError>>`. Three cases:

| Scenario | Behavior |
|----------|----------|
| **NaN / divergence** | Auto-reset by BatchSim (transparent). `data.divergence_detected()` returns true. VecEnv treats this as `done = true` — the env was unrecoverable, so we extract terminal obs (if possible), reset, and continue. |
| **All envs succeed** | Normal path. No errors. |
| **Non-recoverable error** (CholeskyFailed, LuSingular) | VecEnv auto-resets the failed env (same as done). `VecStepResult` includes an `errors: Vec<Option<StepError>>` field so the caller can inspect which envs had physics failures. Training continues — one bad env shouldn't halt a 64-env batch. |

```rust
pub struct VecStepResult {
    pub observations: Tensor,
    pub rewards: Vec<f64>,
    pub dones: Vec<bool>,
    pub truncateds: Vec<bool>,
    pub terminal_observations: Vec<Option<Tensor>>,
    /// Per-env physics errors. `None` = success. `Some(e)` = this env had
    /// a non-recoverable physics error and was auto-reset. Most training
    /// loops can ignore this — it's for diagnostics.
    pub errors: Vec<Option<StepError>>,
}
```

`VecEnv::step()` returns `Err(VecStepError)` only for bridge-level errors
(wrong action tensor shape, etc.), never for per-env physics errors. Physics
errors are per-env and reported in `VecStepResult::errors`.

### 7.5 Auto-Reset Implementation

When an env is done, truncated, or has a physics error, `VecEnv::step()`:
1. Extracts the terminal observation → `terminal_observations[i] = Some(obs)`
   (skipped if the env diverged and state is garbage — `Some` only if
   `!data.divergence_detected()`)
2. Calls `self.batch.reset(i)`
3. Calls `on_reset_fn(model, data, i)` if set
4. Calls `data.forward(model)` to recompute derived quantities
   (if `forward()` fails, the env remains in reset state — clean qpos0 —
   and the error is logged in `VecStepResult::errors[i]`)
5. Extracts the fresh initial observation → `observations[i]`

The `done`/`truncated` flags in VecStepResult refer to the completed episode.
The observation in `observations` is the post-reset initial obs.

## 8. The RL Loop

Putting it all together:

```rust
// ── Build the model ──
let model = Arc::new(load_mjcf("humanoid.xml")?);

// ── Define spaces ──
let obs_space = ObservationSpace::builder()
    .all_qpos()
    .all_qvel()
    .sensor("accelerometer")
    .sensor("gyro")
    .build(&model)?;

let act_space = ActionSpace::builder()
    .all_ctrl()
    .build(&model)?;

// ── Create 64 parallel environments ──
let mut env = VecEnv::builder(model.clone(), 64)
    .observation_space(obs_space)
    .action_space(act_space)
    .reward(|_model, data| {
        let height = data.xpos[1].z;
        let velocity = data.qvel[0];
        velocity - 0.1 * (height - 1.2).powi(2)
    })
    .done(|_model, data| data.xpos[1].z < 0.5)
    .truncated(|_model, data| data.time > 10.0)
    .sub_steps(10)  // 500 Hz physics, 50 Hz actions
    .build()?;

// ── Training loop ──
let mut obs = env.reset_all()?;  // [64, obs_dim]
loop {
    let actions = policy.forward(&obs);              // [64, act_dim]
    let result = env.step(&actions)?;

    buffer.push(&obs, &actions, &result);

    if buffer.len() >= batch_size {
        policy.update(&buffer);
    }

    obs = result.observations;  // [64, obs_dim] — includes auto-reset obs
}
```

## 9. Sub-stepping

Many RL tasks use a coarser action rate than the physics timestep. For
example: physics at 500 Hz, actions at 50 Hz → 10 sub-steps per action.

`sub_steps` is set at env construction time. Each call to `env.step(action)`
applies the action once and then calls `data.step(&model)` `sub_steps` times.
The reward/done/truncated functions see the final state after all sub-steps.

This matches MuJoCo's `n_sub_steps` parameter in dm_control.

### SimEnv: early termination

`SimEnv` checks `done_fn` after each sub-step. If true, remaining sub-steps
are skipped. Reward is computed from the terminal state. This is cheap
(single env, one branch per sub-step) and avoids post-terminal corruption.

### VecEnv: evaluate at end only

`VecEnv` runs all N sub-steps for all envs via `batch.step_all()`, then
evaluates done/truncated once at the final state. **No per-env early exit.**

Why: `step_all()` steps every env in one parallel dispatch. Per-env early
exit would require either (a) masking individual envs out of the batch on
each sub-step (breaking the single `par_iter_mut` dispatch) or (b) running
individual `data.step()` calls (losing batch parallelism). Neither is worth
the complexity for v1.

This matches the standard approach in dm_control and Gymnasium MuJoCo
wrappers: all sub-steps run unconditionally, done is evaluated at the end.
In practice, physics tasks don't "recover" from terminal states within a
few sub-steps — a humanoid that falls below height 0.5 at sub-step 3 will
still be below 0.5 at sub-step 10.

If a future task requires per-env early exit in VecEnv, the growth path is:
replace the `step_all()` × N loop with a `step_where(awake_mask)` method
on BatchSim that skips sleeping/done envs. This is additive to BatchSim,
not a redesign of VecEnv.

## 10. Error Handling

### Error types

| Type | Source | Meaning |
|------|--------|---------|
| `TensorError` | `Tensor::try_from_*` | Shape mismatch: `data.len() != product(shape)`. For external input validation. Internal code uses panicking `from_slice` instead (§5.1). |
| `SpaceError` | Builders | Range out of bounds, sensor name not found, dimension mismatch. Returned by `ObservationSpaceBuilder::build()`, `ActionSpaceBuilder::build()`. |
| `EnvError` | Env builders | Missing required field (no obs space, no reward fn). Returned by `SimEnvBuilder::build()`, `VecEnvBuilder::build()`. |
| `VecStepError` | `VecEnv::step()` | Bridge-level error: wrong action shape, wrong batch size. NOT per-env physics errors. |
| `ResetError` | `reset()` / `reset_all()` | `forward()` failed after reset + on_reset hook. Rare — indicates the on_reset hook put Data into an invalid state. |
| `StepError` | sim-core | Per-env physics error. In VecEnv, reported per-env in `VecStepResult::errors`. In SimEnv, propagated as `Result` from `SimEnv::step()`. |

### SimEnv error propagation

`SimEnv::step()` returns `Result<StepResult, StepError>`. Physics errors
propagate directly — the caller decides whether to reset and retry.

### VecEnv error propagation

See §7.4. Per-env physics errors never halt the batch. They're reported
in `VecStepResult::errors` and the failed env is auto-reset. Only
bridge-level errors (wrong tensor shape) return `Err(VecStepError)`.

Zero `unwrap` in library code. All fallible operations return `Result`.

## 11. Reproducibility and Seeding

The bridge and sim are fully deterministic. There is no internal RNG
state anywhere in `Model`, `Data`, `BatchSim`, or the bridge.

**The only source of non-determinism is the user's `on_reset` hook.**

Contract:
- The bridge guarantees: given identical actions and identical initial
  state, `VecEnv::step()` produces bit-identical results regardless of
  thread count or scheduling order (inherited from BatchSim's determinism
  guarantee).
- The user guarantees: if they want reproducible training runs, they must
  seed their `on_reset` RNG deterministically.

**Recommended pattern for reproducible VecEnv:**

```rust
use rand::SeedableRng;
use rand::rngs::StdRng;

// Master seed → per-env child seeds (deterministic)
let master_seed: u64 = 42;
let mut rngs: Vec<StdRng> = (0..n_envs)
    .map(|i| StdRng::seed_from_u64(master_seed + i as u64))
    .collect();

VecEnv::builder(model, n_envs)
    .on_reset(move |_model, data, env_idx| {
        let rng = &mut rngs[env_idx];
        data.qpos[0] += rng.gen_range(-0.1..0.1);
    })
    .build()?;
```

Because resets are sequential (§7.3) and the per-env RNG is indexed by
`env_idx`, this produces identical randomization regardless of which envs
reset on which step.

## 12. Performance

### Targets

The bridge must not be the bottleneck. Physics stepping dominates wall time;
the bridge (extraction, injection, reward/done evaluation) should be <5%
of step time.

| Benchmark | Target | Notes |
|-----------|--------|-------|
| `extract()` single env | <1 µs | Flat copy + f64→f32 cast for ~100 floats |
| `apply()` single env | <500 ns | f32→f64 cast + clamp for ~20 floats |
| `extract_batch()` 64 envs | <50 µs | Single contiguous write, no per-env alloc |
| `VecEnv::step()` overhead | <5% of total | Extraction + injection + reward/done eval |
| VecEnv 64 pendulums, 1 sub-step | >100K action-steps/sec | Physics-bound, not bridge-bound |
| VecEnv 64 humanoids, 10 sub-steps | >10K action-steps/sec | Complex model, still physics-bound |

These are first-pass targets. We measure in Phase 4b and adjust if needed.
The key invariant: the bridge should never appear in profiling hot spots.

### Allocation budget

| Operation | Allocations per call |
|-----------|---------------------|
| `Tensor::zeros` | 2 (data vec + shape vec) |
| `extract()` | 2 (one Tensor) |
| `extract_batch()` | 2 (one Tensor) |
| `apply()` / `apply_batch()` | 0 |
| `VecEnv::step()` | 2 (observations Tensor) + N (terminal_observations for done envs) + 4 (result vecs) |

v1 allocates per step. Growth path (§14): VecEnv pre-allocates internal
buffers, `step()` fills in-place, zero-alloc hot path.

## 13. What This Crate Does NOT Do

- **No policy implementations** — that's the RL library's job.
- **No autodiff** — that's the autodiff crate's job. Tensor is a dumb buffer.
- **No GPU tensor ops** — Tensor lives on CPU. GPU acceleration lives in
  sim-gpu (compute shaders) and future autodiff (GPU kernels).
- **No observation normalization** — running mean/std normalization belongs
  in the RL library's policy preprocessing, not in the bridge.
- **No domain randomization primitives** — the `on_reset` hook gives users
  full control. The bridge doesn't prescribe a DR strategy.
- **No logging/metrics** — the user instruments their own training loop.
- **No Bevy dependency** — this is Layer 0.

## 14. Growth Path

This is v1. The conscious decisions here create clean growth paths:

| Future capability | How it fits |
|---|---|
| **Autodiff** | Replace `Tensor` internals with a tape-backed tensor. `ObservationSpace::extract()` and `ActionSpace::apply()` become differentiable ops. External API unchanged. |
| **GPU tensors** | `Tensor` gains a `Device` enum (Cpu/Gpu). Extraction/injection kernels move to compute shaders. Builder API unchanged. |
| **Custom extractors** | `Extractor` enum grows new variants (joint torques, contact normals, etc.) without breaking existing builders. |
| **Multi-model VecEnv** | Requires a `HeterogeneousVecEnv` — different from `VecEnv` (which assumes shared Model). Separate type, not a generalization. |
| **State serialization** | Add `serde` feature to sim-core's Data (additive, not structural). Tensor already has `Vec<f32>` — trivially serializable. |
| **Curriculum learning** | User switches reward/done closures between phases. Or: `VecEnv` gains `set_reward()` / `set_done()` methods. |
| **Pre-allocated buffers** | v1 allocates VecStepResult per step. v2: VecEnv owns internal buffers, `step()` fills them in-place, returns a borrow or lightweight copy. Zero-alloc hot path. |

## 15. Implementation Plan

### Phase 1: Crate scaffold + Tensor + TensorSpec
- Cargo.toml, lib.rs, module structure
- `tensor.rs`: Tensor struct, constructors, f64→f32 conversion, shape validation
- `tensor.rs`: TensorSpec struct
- `tensor.rs`: `row()` / `row_mut()` for batched tensor access
- Tests: shape arithmetic, f64→f32 precision, row access, edge cases

### Phase 2: ObservationSpace + ActionSpace
- `space.rs`: Extractor enum, ObservationSpace, builder, `extract()`, `extract_batch()`
- `space.rs`: Injector enum, ActionSpace, builder, `apply()`, `apply_batch()`
- `error.rs`: SpaceError
- Tests: round-trip (extract → verify against direct Data field access),
  builder validation errors, range boundary checks, all extractor variants,
  sensor-by-name resolution, ctrl clamping, batch extract/apply

### Phase 3: Environment trait + SimEnv
- `env.rs`: Environment trait, SimEnv struct, SimEnvBuilder
- `error.rs`: EnvError, VecStepError, ResetError
- Tests: single-env episode lifecycle, sub-stepping correctness, early
  termination during sub-steps, reward/done/truncated correctness,
  on_reset hook + forward() recomputation, builder validation

### Phase 4: VecEnv
- `vec_env.rs`: VecEnv, VecEnvBuilder, VecStepResult
- Tests: batched step matches N sequential SimEnv steps (bit-exact),
  auto-reset semantics (terminal_observations populated correctly),
  per-env action isolation, done/truncated flag correctness,
  on_reset with env_index, sub-stepping, physics error auto-reset

### Phase 4b: Benchmarks
- `benches/bridge_benchmarks.rs`: criterion benchmarks
- `extract()` / `extract_batch()` latency (pendulum, humanoid)
- `apply()` / `apply_batch()` latency
- `VecEnv::step()` total throughput: 64 pendulums, 64 humanoids
- Bridge overhead measurement: `VecEnv::step()` vs raw `BatchSim::step_all()`
- Verify targets from §12: bridge <5% of step time, extract <1 µs, etc.
- If targets are missed: profile, fix, re-measure before Phase 5

### Phase 5: Integration examples
- Pendulum swing-up (simplest: 1 hinge joint, 1 actuator, scalar obs)
- Cart-pole balance (classic RL benchmark, 4D obs, 1D action)
- Multi-env throughput demo (64 envs, measure steps/sec)

## 16. Acceptance Criteria

### Correctness
- [ ] `VecEnv::step()` with 64 envs produces bit-exact results vs
      64 sequential `SimEnv::step()` calls
- [ ] Auto-reset populates `terminal_observations` correctly
- [ ] Auto-reset returns post-reset initial obs in `observations`
- [ ] All observation extractors verified: extract → compare to direct
      Data field access (f64→f32 cast matches)
- [ ] Action clamping respects `model.actuator_ctrlrange`
- [ ] Sub-stepping produces identical results to manual N×`step()` calls
- [ ] Early termination during sub-stepping stops at terminal state
- [ ] `on_reset` hook runs after `Data::reset()`, `forward()` recomputes
      derived quantities before observation extraction
- [ ] Sensor-by-name resolution fails gracefully at `build()` time
- [ ] Batched tensor shapes are correct: `[n_envs, obs_dim]` / `[n_envs, act_dim]`
- [ ] Per-env physics errors reported in `VecStepResult::errors`, don't halt batch
- [ ] Diverged envs auto-reset, `terminal_observations[i]` is `None` (state is garbage)
- [ ] Reproducible: same seed + same actions → bit-identical trajectory

### Quality
- [ ] Zero Bevy dependencies in `cargo tree`
- [ ] Zero ML framework dependencies
- [ ] Zero `unwrap`/`expect` in library code
- [ ] Full rustdoc (zero doc warnings)
- [ ] Clippy clean (`-D warnings`)

### Performance
- [ ] `extract()` single env <1 µs (criterion bench)
- [ ] `apply()` single env <500 ns (criterion bench)
- [ ] Bridge overhead <5% of `VecEnv::step()` total time
- [ ] VecEnv 64 pendulums >100K action-steps/sec
