# sim-ml-bridge Spec

> **Status**: Draft v2  
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
    fn step(&mut self, action: &Tensor) -> StepResult;
    fn reset(&mut self) -> Tensor;
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

**`SimEnv::step()` implementation**:
1. `self.act_space.apply(action, &mut self.data)`
2. For `sub_steps` iterations: `self.data.step(&self.model)?`
3. Compute `reward = (self.reward_fn)(&self.model, &self.data)`
4. Compute `done = (self.done_fn)(&self.model, &self.data)`
5. Compute `truncated = (self.truncated_fn)(&self.model, &self.data)`
6. Extract `obs = self.obs_space.extract(&self.data)`
7. Return `StepResult { observation: obs, reward, done, truncated }`

**`SimEnv::reset()` implementation**:
1. `self.data.reset(&self.model)` — standard Data reset (zeros everything)
2. If `on_reset_fn` is set: `(self.on_reset_fn)(&self.model, &mut self.data)` — domain randomization
3. `self.data.forward(&self.model)?` — recompute derived quantities from modified state
4. Extract and return initial observation

Step 3 is important: if `on_reset` modifies `qpos` (e.g., random initial angles),
the derived quantities (`xpos`, `xquat`, `sensordata`) must be recomputed
before extracting the observation. `forward()` does this without integrating.

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
    on_reset_fn: Option<Arc<dyn Fn(&Model, &mut Data, usize) + Send + Sync>>,
    sub_steps: usize,
}
```

**Core API**:
```rust
impl VecEnv {
    pub fn builder(model: Arc<Model>, n_envs: usize) -> VecEnvBuilder;

    /// Number of parallel environments.
    pub fn n_envs(&self) -> usize;

    /// Step all environments.
    ///
    /// `actions` has shape [n_envs, act_dim]. Row i is applied to env i.
    /// Environments where done or truncated is true are automatically
    /// reset after the step.
    pub fn step(&mut self, actions: &Tensor) -> VecStepResult;

    /// Reset all environments, returning initial observations.
    /// Shape: [n_envs, obs_dim].
    pub fn reset_all(&mut self) -> Tensor;

    /// Access underlying BatchSim for advanced use.
    pub fn batch(&self) -> &BatchSim;
    pub fn batch_mut(&mut self) -> &mut BatchSim;

    /// Access shared model.
    pub fn model(&self) -> &Model;
}
```

**`VecEnv::step()` implementation**:
1. Scatter actions: for each env `i`, `self.act_space.apply(actions.row(i), env_i)`
2. For `sub_steps` iterations: `self.batch.step_all()`
3. For each env: compute reward, done, truncated
4. For done/truncated envs: extract terminal observation, then reset
5. Extract batched observation `[n_envs, obs_dim]` from all envs
6. Return `VecStepResult`

**`on_reset_fn` for VecEnv**: takes `(model, data, env_index)` — the env
index lets the closure use per-env RNG seeds or deterministic domain
randomization schedules. The closure is `Fn` (not `FnMut`) + `Send + Sync`
because VecEnv may reset multiple envs and the closure must be callable from
any context. Per-env mutable state (RNG) should live outside the closure
or use interior mutability.

**Auto-reset semantics**: when an env is done or truncated, `VecEnv::step()`:
1. Extracts the terminal observation → `terminal_observations[i] = Some(obs)`
2. Calls `self.batch.reset(i)`
3. Calls `on_reset_fn` if set
4. Calls `forward()` on the reset env to recompute derived quantities
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
let mut obs = env.reset_all();  // [64, obs_dim]
loop {
    let actions = policy.forward(&obs);           // [64, act_dim]
    let result = env.step(&actions);

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

**Early termination during sub-stepping**: if `done_fn` returns true after
sub-step `k < sub_steps`, remaining sub-steps are skipped. The reward is
computed from the terminal state. This prevents wasting compute on a dead
env and avoids post-terminal state corruption.

## 10. Error Handling

- `SpaceError` — returned by builders when ranges are out of bounds, sensor
  names aren't found, or dimensions don't match the model.
- `EnvError` — returned by `SimEnvBuilder::build()` / `VecEnvBuilder::build()`
  for missing required fields (no observation space, no reward function, etc.).
- `StepError` — physics errors from sim-core. In `VecEnv`, NaN/divergence
  triggers auto-reset (inherited from BatchSim). Non-recoverable errors
  (`CholeskyFailed`, `LuSingular`) propagate as `Result`.

Zero `unwrap` in library code. All fallible operations return `Result`.

## 11. What This Crate Does NOT Do

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

## 12. Growth Path

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

## 13. Implementation Plan

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
- `error.rs`: EnvError
- Tests: single-env episode lifecycle, sub-stepping correctness, early
  termination during sub-steps, reward/done/truncated correctness,
  on_reset hook + forward() recomputation, builder validation

### Phase 4: VecEnv
- `vec_env.rs`: VecEnv, VecEnvBuilder, VecStepResult
- Tests: batched step matches N sequential SimEnv steps (bit-exact),
  auto-reset semantics (terminal_observations populated correctly),
  per-env action isolation, done/truncated flag correctness,
  on_reset with env_index, sub-stepping

### Phase 5: Integration examples
- Pendulum swing-up (simplest: 1 hinge joint, 1 actuator, scalar obs)
- Cart-pole balance (classic RL benchmark, 4D obs, 1D action)
- Multi-env throughput demo (64 envs, measure steps/sec)

## 14. Acceptance Criteria

- [ ] Zero Bevy dependencies in `cargo tree`
- [ ] Zero ML framework dependencies
- [ ] Zero `unwrap`/`expect` in library code
- [ ] Full rustdoc (zero doc warnings)
- [ ] Clippy clean (`-D warnings`)
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
