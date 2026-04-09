# Policy Persistence — Artifact & Checkpoint Spec

> Build foundational infrastructure for saving, loading, and transferring
> learned behavior across the ML domain. Not a bolt-on — a structural part
> of every trait, every algorithm, every policy type. The currency of the
> ML layer.

**Status**: v9 — Phase 2 complete (commit `dee449b`), Phase 3 next
**Crate**: `sim-ml-bridge`
**New dependencies**: `serde`, `serde_json`

---

## 1. Design principles

These mirror how logging was built — structural, universal, extensible.

| Principle | What it means |
|-----------|---------------|
| **Structural** | Artifact extraction is on the trait, not bolted on. You can't implement Algorithm without implementing `policy_artifact()`. Same as `on_epoch` being in the `train()` signature — not optional. |
| **Universal** | Every policy type, every algorithm, same artifact format. CEM, TD3, SAC all produce the same `PolicyArtifact`. You never ask "which serialization does this algorithm use?" |
| **Self-describing** | An artifact file contains everything needed to reconstruct a live policy and understand where it came from. No external schema, no companion files, no "which hidden_dim was this?" guesswork. |
| **Forward-compatible** | Adding new policy kinds, new provenance fields, new checkpoint data never breaks existing readers. Unknown JSON fields are silently ignored. Version field gates structural changes. |
| **Human-inspectable** | `cat policy.artifact.json` tells you the architecture, the weights, the training history. No hex editors, no binary decoders. Debuggability is non-negotiable. |
| **Composable** | Artifacts are the transfer mechanism between training stages. Train with CEM → save → load as initialization for TD3 → save → deploy in Bevy. Each stage appends provenance. |
| **No dead code on traits** | Every trait method has a real implementation. No `todo!()` bodies, no panic stubs. If it goes on the trait, it ships with working code for every implementor. |

---

## 2. Problem statement

After `Algorithm::train()` returns, the trained policy is locked inside
the algorithm's `Box<dyn Policy>`. This is the equivalent of a training
loop that prints metrics to stdout but doesn't return them.

**What's missing:**

| Gap | Impact |
|-----|--------|
| No weight extraction | Can't inspect what was learned |
| No serialization | Weights exist only in RAM — lost on process exit |
| No architecture metadata | Raw `Vec<f64>` is meaningless without knowing obs_dim, hidden_dims, etc. |
| No provenance | No record of which algorithm, task, seed, or hyperparams produced the weights |
| No reconstruction | Can't go from saved weights back to a live `forward()`-capable policy |
| No visual replay | Can't render a trained policy controlling an arm in Bevy |
| No curriculum/transfer | Can't load weights from one training run as initialization for another |
| No checkpointing | Can't resume a long training run across sessions |

---

## 3. What is "learned behavior"?

Five things come out of training, at three tiers of scope:

### Tier 1: Artifact (portable, algorithm-agnostic, sufficient for deployment)

| Component | What | Who needs it |
|-----------|------|-------------|
| **Policy weights** | Flat `Vec<f64>` — the learned obs → action mapping | Everyone |
| **Architecture recipe** | obs_dim, act_dim, hidden_dims, activation, obs_scale, stochastic | Anyone reconstructing a live policy |
| **Training provenance** | Algorithm, task, seed, final reward, full training curve | Analysis, comparison, audit, curriculum lineage |

### Tier 2: Checkpoint (algorithm-specific, needed to resume training)

| Component | What | Who needs it |
|-----------|------|-------------|
| **Critic state** | ValueFn / QFunction weights + architectures | Resume PPO (V), TD3/SAC (Q1/Q2 + targets) |
| **Optimizer momentum** | Adam's `m`, `v`, `t` per optimizer instance | Resume without regression (warm momentum) |

### Tier 3: Full state (deferred — large, complex)

| Component | What | Who needs it |
|-----------|------|-------------|
| **Replay buffer** | Stored transitions for off-policy | Resume TD3/SAC without re-collecting data |
| **RNG state** | Exact random number generator state | Bit-exact reproducibility across resume |

**Scope of this spec**: Tiers 1 and 2 fully implemented — types, trait
methods, and real implementations for every algorithm. Tier 3 types
defined, implementations deferred (replay buffers are large and the
data model needs real-world checkpoint sizes to inform the design).

---

## 4. Core types

### 4.1 NetworkKind — shared architecture enum

One enum shared by policies, value functions, and Q-functions. The
distinction between "a Linear policy" and "a Linear value function"
is in the descriptor type, not the kind.

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum NetworkKind {
    Linear,
    Mlp,
    Autograd,
}
```

`#[non_exhaustive]` — future kinds (Cnn, Rnn, Transformer,
MixtureOfExperts) can be added without breaking existing serialized files
or match arms.

**Implementation note**: Within the defining crate, the compiler sees all
variants and warns about unreachable wildcard patterns. Functions that
match on `NetworkKind` need `#[allow(unreachable_patterns)]` to suppress
this — the wildcard IS needed for downstream crates and future variants.

### 4.2 PolicyDescriptor — the recipe card

Enough information to reconstruct an empty policy with the same structure.

```rust
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PolicyDescriptor {
    /// Which concrete type to construct.
    pub kind: NetworkKind,
    /// Observation dimensionality.
    pub obs_dim: usize,
    /// Action dimensionality.
    pub act_dim: usize,
    /// Hidden layer sizes. [] for Linear, [H] for Mlp, [H1, H2, ...] for Autograd.
    pub hidden_dims: Vec<usize>,
    /// Activation function for hidden layers.
    pub activation: Activation,
    /// Per-observation-dimension scaling factors.
    pub obs_scale: Vec<f64>,
    /// Whether this policy has learned exploration (log_std as parameter).
    pub stochastic: bool,
}
```

### 4.3 NetworkDescriptor — recipe card for critics

Same pattern as PolicyDescriptor but for ValueFn and QFunction types.
Uses the same `NetworkKind` enum. No `stochastic` flag — critics don't
have learned exploration.

```rust
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct NetworkDescriptor {
    /// Which concrete type to construct.
    pub kind: NetworkKind,
    /// Observation dimensionality.
    pub obs_dim: usize,
    /// Action dimensionality (Some for Q-functions that take (obs, action), None for V-functions).
    pub act_dim: Option<usize>,
    /// Hidden layer sizes.
    pub hidden_dims: Vec<usize>,
    /// Activation function for hidden layers.
    pub activation: Activation,
    /// Per-observation-dimension scaling factors.
    pub obs_scale: Vec<f64>,
}
```

**Conversion**: `impl From<PolicyDescriptor> for NetworkDescriptor` maps
`act_dim` to `Some(act_dim)` and drops `stochastic`. Used by TD3's
`checkpoint()` to store the target policy (a `DifferentiablePolicy` that
returns `PolicyDescriptor`) as a `NetworkSnapshot` (which uses
`NetworkDescriptor`).

### 4.4 PolicyArtifact — the deployable brain

The foundational currency of the ML domain.

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PolicyArtifact {
    /// Format version — increment on structural changes.
    pub version: u32,
    /// Architecture — enough to reconstruct an empty policy.
    pub descriptor: PolicyDescriptor,
    /// Learned weights — length must match descriptor's implied param count.
    pub params: Vec<f64>,
    /// How these weights were produced. None for hand-crafted initial weights.
    pub provenance: Option<TrainingProvenance>,
}
```

### 4.5 TrainingProvenance — the lineage

Provenance is **built by the caller, not the algorithm.** The algorithm
knows the policy state. The caller (Competition runner, Bevy example)
knows the task name, seed, hyperparams, and has the returned metrics.
See §5.4 and §7 for provenance assembly patterns.

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingProvenance {
    /// Algorithm that produced these weights (e.g., "CEM", "TD3").
    pub algorithm: String,
    /// Task trained on (e.g., "reaching-6dof").
    pub task: String,
    /// RNG seed used for training.
    pub seed: u64,
    /// Number of epochs completed.
    pub epochs_trained: usize,
    /// Final epoch's mean reward.
    pub final_reward: f64,
    /// Total environment steps across all epochs.
    pub total_steps: usize,
    /// Total wall-clock training time in milliseconds.
    pub wall_time_ms: u64,
    /// ISO 8601 timestamp of when training completed.
    pub timestamp: String,
    /// Algorithm-specific hyperparameters.
    /// Mirrors the EpochMetrics.extra pattern — append-only, no schema coupling.
    /// Integer hyperparams (batch_size, etc.) stored as f64 for consistency
    /// with the existing EpochMetrics.extra convention.
    pub hyperparams: BTreeMap<String, f64>,
    /// Full training curve — every epoch's metrics.
    pub metrics: Vec<EpochMetrics>,
    /// If these weights were initialized from a prior artifact (curriculum, transfer, fine-tune).
    /// Forms a linked-list provenance chain: current → parent → grandparent → ...
    pub parent: Option<Box<TrainingProvenance>>,
}
```

The `parent` field is the key future-proofing decision. Curriculum learning:
train on easy task → save → load as initialization → train on hard task. The
hard-task provenance's `parent` points to the easy-task provenance. Full
chain-of-custody, no matter how many training stages.

### 4.6 ArtifactError — dedicated error type

```rust
#[derive(Debug, thiserror::Error)]
pub enum ArtifactError {
    #[error("unsupported artifact version {found} (max supported: {max})")]
    UnsupportedVersion { found: u32, max: u32 },

    #[error("unknown network kind — artifact may have been written by a newer version")]
    UnknownKind,

    #[error("param count mismatch: descriptor implies {expected}, artifact has {actual}")]
    ParamCountMismatch { expected: usize, actual: usize },

    #[error("obs_scale length {actual} doesn't match obs_dim {expected}")]
    ObsScaleMismatch { expected: usize, actual: usize },

    #[error("hidden_dims must be empty for Linear kind, got {0} layers")]
    LinearHiddenDims(usize),

    #[error("hidden_dims must be non-empty for {kind:?} kind")]
    MissingHiddenDims { kind: NetworkKind },

    #[error("unsupported combination: kind={kind:?} with stochastic={stochastic}")]
    UnsupportedCombination { kind: NetworkKind, stochastic: bool },

    #[error("non-finite f64 value in field `{field}`")]
    NonFiniteValue { field: String },

    #[error(transparent)]
    Io(#[from] std::io::Error),

    #[error(transparent)]
    Json(#[from] serde_json::Error),
}
```

Used by `save()`, `load()`, `to_policy()`, and `validate()`.

**No `Clone` derive** — `std::io::Error` and `serde_json::Error` don't
implement `Clone`. This means callers can't clone `ArtifactError` values,
which is fine — errors are consumed, not shared.

`UnsupportedCombination` is returned by `to_policy()` for valid `NetworkKind`
values that don't have a concrete type for the requested `stochastic` mode
(e.g., `Mlp` + `stochastic: true` — `MlpStochasticPolicy` doesn't exist).

`NonFiniteValue` is returned by `validate()` and `save()` when any `f64`
field (params, metrics, etc.) contains NaN or Infinity (see §6.6).

### 4.7 TrainingCheckpoint — the resume point

Full training state for resuming across sessions. Every algorithm
implements `checkpoint()` with a real implementation — no stubs.

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingCheckpoint {
    /// Which algorithm produced this checkpoint (e.g., "CEM", "TD3").
    /// Needed for reconstruction — tells the caller which from_checkpoint() to call.
    pub algorithm_name: String,
    /// The trained policy at this point.
    pub policy_artifact: PolicyArtifact,
    /// Critic networks (V for PPO; Q1, Q2, targets for TD3/SAC).
    pub critics: Vec<NetworkSnapshot>,
    /// Optimizer states (one per optimizer instance).
    pub optimizer_states: Vec<OptimizerSnapshot>,
    /// Algorithm-specific scalar state (noise_std, sigma, alpha, etc.).
    pub algorithm_state: BTreeMap<String, f64>,
    // Future: replay_buffer: Option<ReplayBufferSnapshot>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkSnapshot {
    /// Role in the algorithm (e.g., "value", "q1", "q2_target").
    pub role: String,
    /// Architecture descriptor.
    pub descriptor: NetworkDescriptor,
    /// Learned weights.
    pub params: Vec<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizerSnapshot {
    /// Which network this optimizer trains (e.g., "actor", "q1", "value").
    pub role: String,
    /// Optimizer configuration at time of snapshot.
    /// Uses the existing OptimizerConfig enum directly — no duplicate type.
    pub config: OptimizerConfig,
    /// First moment estimates (Adam m).
    pub m: Vec<f64>,
    /// Second moment estimates (Adam v).
    pub v: Vec<f64>,
    /// Step count (Adam t).
    pub t: usize,
}
```

Note: `OptimizerSnapshot.config` uses the existing `OptimizerConfig` enum
directly (with serde derives added). No separate snapshot config type.

**Implementation note (infinity serde)**: `OptimizerConfig::Adam.max_grad_norm`
defaults to `f64::INFINITY` (no clipping). `serde_json` rejects
non-finite f64 values, so `max_grad_norm` has custom `serialize_with` /
`deserialize_with` attributes: serializes infinity as `null`, deserializes
`null` back to `f64::INFINITY`.

**Checkpoint contents per algorithm:**

| Algorithm | policy_artifact | critics (NetworkSnapshot vec) | optimizers | algorithm_state |
|-----------|----------------|-------------------------------|------------|-----------------|
| CEM | actor | — | — | `noise_std` |
| REINFORCE | actor | — | actor | `sigma` |
| PPO | actor | value | actor, value | `sigma` |
| TD3 | actor | actor_target, q1, q2, q1_target, q2_target | actor, q1, q2 | — |
| SAC | actor | q1, q2, q1_target, q2_target | actor, q1, q2 | `log_alpha`, `alpha_lr` |

**TD3 target actor**: Stored in the `critics` Vec as a `NetworkSnapshot`
with role `"actor_target"`, not in `policy_artifact`. `NetworkDescriptor`
(not `PolicyDescriptor`) is sufficient because target actors are always
deterministic — the `stochastic` flag is meaningless for target networks.

**Checkpoint reconstruction note**: Each algorithm's `from_checkpoint()`
must reconstruct the correct trait object type from `NetworkSnapshot`
descriptors. TD3 needs `Box<dyn DifferentiablePolicy>` for both actor and
actor_target; SAC needs `Box<dyn StochasticPolicy>` for its actor. The
`NetworkSnapshot` descriptor (plus `algorithm_name` on the checkpoint)
provides enough information for each algorithm's constructor to select the
right concrete type.

**Reconstruction pattern:**

Checkpoints capture learned state. Hyperparams are configuration the
caller provides — you might want to resume with a different learning
rate (annealing) or different batch size:

```rust
impl Td3 {
    pub fn from_checkpoint(
        checkpoint: &TrainingCheckpoint,
        optimizer_config: OptimizerConfig,
        hyperparams: Td3Hyperparams,
    ) -> Result<Self, ArtifactError> {
        // Reconstruct policy from checkpoint.policy_artifact
        // Reconstruct critics from checkpoint.critics
        // Reconstruct optimizers from checkpoint.optimizer_states
        // Apply hyperparams
    }
}
```

Each algorithm has its own `from_checkpoint()` constructor (not on the
trait — returns concrete `Self`, needs algorithm-specific hyperparams).

### 4.8 Checkpoint save/load

```rust
impl TrainingCheckpoint {
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), ArtifactError> {
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(path, json)?;
        Ok(())
    }

    pub fn load(path: impl AsRef<Path>) -> Result<Self, ArtifactError> {
        let json = std::fs::read_to_string(path)?;
        let checkpoint: Self = serde_json::from_str(&json)?;
        // Version check via the embedded policy artifact
        if checkpoint.policy_artifact.version > CURRENT_VERSION {
            return Err(ArtifactError::UnsupportedVersion {
                found: checkpoint.policy_artifact.version,
                max: CURRENT_VERSION,
            });
        }
        Ok(checkpoint)
    }
}
```

### 4.9 State promotion — structural prerequisite

The `checkpoint()` method has signature `fn checkpoint(&self) ->
TrainingCheckpoint`. For `&self` to have access to mutable training state,
that state must be stored on the algorithm struct — not as local variables
inside `train()`.

**Current situation**: All 5 algorithms build optimizers as local variables
inside `train()`. Mutable scalars (noise_std, sigma, log_alpha) are also
locals. After `train()` returns, this state is dropped.

**Required change**: Before `checkpoint()` can work, each algorithm must
promote optimizer instances and mutable scalars from `train()` locals to
struct fields. This is a code-level prerequisite — not a new type or trait,
but a structural change to algorithm internals.

**Promoted state per algorithm:**

| Algorithm | New struct fields | Initialized in |
|-----------|-------------------|----------------|
| CEM | `noise_std: f64` | `new()` from `hp.noise_std` |
| REINFORCE | `optimizer: Box<dyn Optimizer>`, `sigma: f64` | `new()` from `optimizer_config.build(n_params)`, `hp.sigma_init` |
| PPO | `actor_opt: Box<dyn Optimizer>`, `critic_opt: Box<dyn Optimizer>`, `sigma: f64` | `new()` from config + policy/value n_params, `hp.sigma_init` |
| TD3 | `actor_opt: Box<dyn Optimizer>`, `q1_opt: Box<dyn Optimizer>`, `q2_opt: Box<dyn Optimizer>` | `new()` from config + network n_params |
| SAC | `actor_opt: Box<dyn Optimizer>`, `q1_opt: Box<dyn Optimizer>`, `q2_opt: Box<dyn Optimizer>`, `log_alpha: f64` | `new()` from config + network n_params, `hp.alpha_init.ln()` |

Optimizers can be built in `new()` because `n_params` is known at
construction — every algorithm already receives its policy (and critics)
as constructor arguments.

**Semantic change**: Calling `train()` twice on the same algorithm instance
now continues from previous state (optimizer momentum, sigma, noise_std,
log_alpha) instead of resetting. This is intentional — it matches the
checkpoint mental model: resuming training means continuing from where you
left off, not restarting. If a fresh start is needed, construct a new
algorithm instance.

**Implementation note (borrow splitting)**: REINFORCE and PPO use a `&mut`
closure passed to `collect_episodic_rollout()`. The closure captures
`self.policy` and needs `sigma`. After promotion, both are on `self`,
creating a borrow conflict. The solution: copy `sigma` to a local before
the closure (`let sigma = self.sigma;`), use the local inside the closure,
then update `self.sigma` after the closure returns. This matches the
original code pattern (sigma was already a local) and avoids any
restructuring of the rollout API.

**Implementation note (dead_code)**: After promotion, `optimizer_config`
fields on REINFORCE, PPO, TD3, and SAC are no longer read at runtime
(optimizers are built once in `new()`). The fields are kept and annotated
with `#[allow(dead_code)]` for introspection and `from_checkpoint()`
reconstruction.

---

## 5. Trait surgery

### 5.1 Policy trait — add `descriptor()`

```rust
pub trait Policy: Send + Sync {
    fn n_params(&self) -> usize;
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);
    fn forward(&self, obs: &[f32]) -> Vec<f64>;
    fn forward_batch(&self, obs_batch: &[f32], obs_dim: usize) -> Vec<f64> { ... }

    /// Describes this policy's architecture — enough to reconstruct
    /// an empty policy with the same structure.
    fn descriptor(&self) -> PolicyDescriptor;
}
```

5 implementations: LinearPolicy, MlpPolicy, AutogradPolicy,
LinearStochasticPolicy, AutogradStochasticPolicy. Each is ~10 lines
returning a struct literal from stored fields or derivable values.

**Implementation notes per type:**
- **LinearPolicy** / **LinearStochasticPolicy**: All descriptor fields
  are stored directly. LinearPolicy has no activation field (purely linear,
  no hidden layer) — `descriptor()` returns `Activation::Tanh` (unused
  since `hidden_dims` is empty).
- **MlpPolicy**: Has no `activation` field — hidden layer activation is
  hardcoded to `tanh`. `descriptor()` returns `activation: Activation::Tanh`.
- **AutogradPolicy** / **AutogradStochasticPolicy**: No `obs_dim` field —
  derived from `self.obs_scale.len()`. Hidden dims are not stored directly
  as `Vec<usize>` — they are derived from `layer_offsets: Vec<LayerOffsets>`
  where each `LayerOffsets` has `in_dim` and `out_dim`. `descriptor()`
  reconstructs `hidden_dims` from intermediate layer dimensions.

### 5.2 ValueFn trait — add `descriptor()`

```rust
pub trait ValueFn: Send + Sync {
    // ... existing methods unchanged ...
    fn descriptor(&self) -> NetworkDescriptor;
}
```

3 implementations: LinearValue, MlpValue, AutogradValue.

### 5.3 QFunction trait — add `descriptor()`

```rust
pub trait QFunction: Send + Sync {
    // ... existing methods unchanged ...
    fn descriptor(&self) -> NetworkDescriptor;
}
```

3 implementations: LinearQ, MlpQ, AutogradQ.

### 5.4 Algorithm trait — add `policy_artifact()` and `checkpoint()`

```rust
pub trait Algorithm: Send {
    fn name(&self) -> &'static str;

    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics>;             // UNCHANGED

    /// Extract the current policy as a portable artifact.
    ///
    /// Returns a **bare** artifact: descriptor + params, provenance = None.
    /// The caller attaches provenance — the algorithm knows the policy state
    /// but not the task name, seed, or training context.
    ///
    /// Can be called after train() for the final policy, or at any time
    /// for intermediate snapshots.
    fn policy_artifact(&self) -> PolicyArtifact;

    /// Extract full training state for later resumption.
    ///
    /// Includes policy, critics, optimizer momentum — everything needed
    /// to continue training without regression. Each algorithm implements
    /// this with real code that captures its specific internal state.
    ///
    /// Depends on state promotion (§4.9): optimizers and mutable scalars
    /// must be struct fields, not train()-local variables. The `&self`
    /// signature works because promoted fields live on the struct.
    ///
    /// Reconstruction is via per-algorithm `from_checkpoint()` constructors
    /// (not on the trait — needs algorithm-specific hyperparams).
    fn checkpoint(&self) -> TrainingCheckpoint;
}
```

`train()` return type is **unchanged** (`Vec<EpochMetrics>`). The artifact
is accessed via the separate `policy_artifact()` method — more flexible
(callable any time) and non-breaking for existing competition assertions.

5 implementations for `policy_artifact()`: CEM, REINFORCE, PPO, TD3, SAC.
5 implementations for `checkpoint()`: CEM, REINFORCE, PPO, TD3, SAC.
5 `from_checkpoint()` constructors (not on trait, per-algorithm).

**Trait object types per algorithm:**
- CEM stores `Box<dyn Policy>` — calls `descriptor()` directly.
- REINFORCE, PPO, TD3 store `Box<dyn DifferentiablePolicy>` — `descriptor()`
  is accessible through the supertrait chain (`DifferentiablePolicy: Policy`).
- SAC stores `Box<dyn StochasticPolicy>` — `descriptor()` is accessible
  through `StochasticPolicy: DifferentiablePolicy: Policy`.

All five call `self.policy.descriptor()` uniformly — the supertrait
resolution is transparent to the implementation.

### 5.5 Optimizer trait — add `snapshot()` and `load_snapshot()`

```rust
pub trait Optimizer: Send + Sync {
    // ... existing methods unchanged ...

    /// Snapshot the optimizer's internal state (momentum, variance, step count).
    fn snapshot(&self, role: &str) -> OptimizerSnapshot;

    /// Restore optimizer state from a snapshot.
    fn load_snapshot(&mut self, snapshot: &OptimizerSnapshot);
}
```

1 implementation: Adam (exposes `m`, `v`, `t` which are already fields
on the struct). Future optimizers (AdamW, SGD+momentum) implement the
same interface.

---

## 6. Serialization architecture

### 6.1 Dependencies

`serde` and `serde_json` become **required** dependencies of `sim-ml-bridge`.
Not optional, not behind a feature flag. Artifact support is foundational —
same reasoning as `on_epoch` being in the `train()` signature.

### 6.2 Serde derives on existing types

These types gain `#[derive(Serialize, Deserialize)]`:

- `EpochMetrics` (needed for training curve in provenance)
- `Activation` (already `Copy + Clone + Debug + PartialEq + Eq`)
- `OptimizerConfig` (used in `OptimizerSnapshot`)

### 6.3 Format: JSON v1

**Why JSON:**
- Human-inspectable: `cat` the file, understand what's in it
- Forward-compatible: unknown fields silently ignored by serde (no `deny_unknown_fields`)
- Self-describing: no external schema needed
- Standard: every language, every tool can read it
- Debuggable: grep for a reward value, diff two artifacts

**Why not binary (yet):**
- Current policies are tiny: Linear ~20 params, Mlp ~2K, Autograd ~5K
- 5K params x 20 bytes/param (JSON text) = 100KB. Not a concern.
- Binary optimization is an additive future step, not a replacement.

**File extension**: `.artifact.json` (policy artifacts), `.checkpoint.json` (training checkpoints)

**Version field**: `version: u32` in every artifact.
- `load()` checks version after deserialization — returns `ArtifactError::UnsupportedVersion` if too high
- Version bumps only for semantic changes to existing fields
- New optional fields never require a version bump (JSON forward compat)

**Forward-compatibility rules** (hard rules — never relax):
- No type in this spec uses `#[serde(deny_unknown_fields)]`. Unknown fields
  must be silently ignored so a v1 reader can load a v2 file that added new
  fields without error.
- All `Option<T>` fields use `#[serde(default)]`. This ensures a reader
  that predates a new optional field silently defaults it to `None` instead
  of failing on a missing key.

### 6.4 Example serialized artifact

```json
{
  "version": 1,
  "descriptor": {
    "kind": "Autograd",
    "obs_dim": 12,
    "act_dim": 6,
    "hidden_dims": [64, 64],
    "activation": "Tanh",
    "obs_scale": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    "stochastic": false
  },
  "params": [0.0234, -0.1567, 0.0891],
  "provenance": {
    "algorithm": "TD3",
    "task": "reaching-6dof",
    "seed": 42,
    "epochs_trained": 50,
    "final_reward": -1.23,
    "total_steps": 50000,
    "wall_time_ms": 34521,
    "timestamp": "2026-04-08T14:30:00Z",
    "hyperparams": {
      "gamma": 0.99,
      "tau": 0.005,
      "policy_noise": 0.2,
      "batch_size": 64.0
    },
    "metrics": [
      {
        "epoch": 0,
        "mean_reward": -15.2,
        "done_count": 0,
        "total_steps": 1000,
        "wall_time_ms": 680,
        "extra": { "q1_loss": 0.45, "q2_loss": 0.51 }
      }
    ],
    "parent": null
  }
}
```

### 6.6 Finite-value policy

`serde_json` rejects NaN and Infinity by default — `to_string` returns
`Err`, not a JSON file with `null` or `"NaN"`. This is correct behavior:
non-finite values in a policy artifact indicate a training bug, not valid
data.

**Hard rule**: All `f64` values in serialized types must be finite.

- `validate()` checks: every element in `params`, and if provenance is
  present, `final_reward` and every `metrics[i].mean_reward`. Returns
  `ArtifactError::NonFiniteValue { field }` on first non-finite value.
- `save()` calls `validate()` before serialization — a non-finite value
  is caught as a structured error, not a serde panic.
- The 0-epoch case in provenance assembly uses `0.0`, not `f64::NAN`:
  `metrics.last().map_or(0.0, |m| m.mean_reward)`.
- Algorithms must produce finite `mean_reward` values. The existing
  `RunResult::assert_finite()` enforces this at test time; `validate()`
  enforces it at serialization time.

### 6.7 Future binary format (not this spec)

When policies hit millions of params: JSON header (metadata, provenance)
followed by raw `f64` binary blob (weights). Like safetensors but simpler.
Added alongside JSON — existing `.artifact.json` files continue to work.
New extension: `.artifact.bin`. `PolicyArtifact::save()` / `load()` detect
format from extension.

---

## 7. Reconstruction and validation

### 7.1 PolicyArtifact methods

```rust
/// Current artifact format version.
pub const CURRENT_VERSION: u32 = 1;

impl PolicyArtifact {
    /// Reconstruct a live policy from this artifact.
    /// The returned policy is ready for forward() — inference/replay only.
    ///
    /// Returns Err if the artifact has an unsupported version, unknown kind,
    /// or mismatched param count.
    pub fn to_policy(&self) -> Result<Box<dyn Policy>, ArtifactError> {
        self.validate()?;
        // Match on (kind, stochastic) to select constructor:
        //   (Linear, false)   → LinearPolicy
        //   (Linear, true)    → LinearStochasticPolicy
        //   (Mlp,    false)   → MlpPolicy
        //   (Mlp,    true)    → ArtifactError::UnsupportedCombination
        //   (Autograd, false) → AutogradPolicy
        //   (Autograd, true)  → AutogradStochasticPolicy
        //   (unknown,  _)     → ArtifactError::UnknownKind
        // Call set_params() to load weights after construction.
    }

    /// Create an artifact from any live policy (no provenance).
    /// Use this from Bevy examples that implement their own training loops.
    pub fn from_policy(policy: &dyn Policy) -> Self {
        Self {
            version: CURRENT_VERSION,
            descriptor: policy.descriptor(),
            params: policy.params().to_vec(),
            provenance: None,
        }
    }

    /// Attach provenance to this artifact. Returns self for chaining.
    pub fn with_provenance(mut self, provenance: TrainingProvenance) -> Self {
        self.provenance = Some(provenance);
        self
    }

    /// Save to JSON file. Validates before serialization (§6.6).
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), ArtifactError> {
        self.validate()?;
        let json = serde_json::to_string_pretty(self)?;
        std::fs::write(path, json)?;
        Ok(())
    }

    /// Load from JSON file. Validates version on load.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, ArtifactError> {
        let json = std::fs::read_to_string(path)?;
        let artifact: Self = serde_json::from_str(&json)?;
        if artifact.version > CURRENT_VERSION {
            return Err(ArtifactError::UnsupportedVersion {
                found: artifact.version,
                max: CURRENT_VERSION,
            });
        }
        Ok(artifact)
    }

    /// Validate internal consistency.
    pub fn validate(&self) -> Result<(), ArtifactError> {
        // 1. Version check
        if self.version > CURRENT_VERSION {
            return Err(ArtifactError::UnsupportedVersion {
                found: self.version,
                max: CURRENT_VERSION,
            });
        }
        // 2. obs_scale length matches obs_dim
        if self.descriptor.obs_scale.len() != self.descriptor.obs_dim {
            return Err(ArtifactError::ObsScaleMismatch {
                expected: self.descriptor.obs_dim,
                actual: self.descriptor.obs_scale.len(),
            });
        }
        // 3. hidden_dims consistency with kind
        match self.descriptor.kind {
            NetworkKind::Linear => {
                if !self.descriptor.hidden_dims.is_empty() {
                    return Err(ArtifactError::LinearHiddenDims(
                        self.descriptor.hidden_dims.len(),
                    ));
                }
            }
            NetworkKind::Mlp | NetworkKind::Autograd => {
                if self.descriptor.hidden_dims.is_empty() {
                    return Err(ArtifactError::MissingHiddenDims {
                        kind: self.descriptor.kind,
                    });
                }
            }
            // Future kinds — can't validate, but to_policy() will
            // return UnknownKind if it can't reconstruct.
            _ => {}
        }
        // 4. Unsupported (kind, stochastic) combinations
        if matches!(self.descriptor.kind, NetworkKind::Mlp) && self.descriptor.stochastic {
            return Err(ArtifactError::UnsupportedCombination {
                kind: self.descriptor.kind,
                stochastic: self.descriptor.stochastic,
            });
        }
        // 5. param count (compute expected from descriptor, compare to actual)
        let expected = compute_param_count(&self.descriptor);
        if self.params.len() != expected {
            return Err(ArtifactError::ParamCountMismatch {
                expected,
                actual: self.params.len(),
            });
        }
        // 6. Finite-value check (§6.6) — reject NaN/Infinity before serialization
        if self.params.iter().any(|v| !v.is_finite()) {
            return Err(ArtifactError::NonFiniteValue {
                field: "params".into(),
            });
        }
        if let Some(ref prov) = self.provenance {
            if !prov.final_reward.is_finite() {
                return Err(ArtifactError::NonFiniteValue {
                    field: "provenance.final_reward".into(),
                });
            }
            for (i, m) in prov.metrics.iter().enumerate() {
                if !m.mean_reward.is_finite() {
                    return Err(ArtifactError::NonFiniteValue {
                        field: format!("provenance.metrics[{i}].mean_reward"),
                    });
                }
            }
        }
        Ok(())
    }
}
```

### 7.1.1 compute_param_count() formula

Used by `validate()` to verify that `params.len()` matches the descriptor's
implied parameter count. Match on `(kind, stochastic)`:

```rust
fn compute_param_count(d: &PolicyDescriptor) -> usize {
    match (d.kind, d.stochastic) {
        // W[act_dim × obs_dim] + b[act_dim]
        (NetworkKind::Linear, false) => d.obs_dim * d.act_dim + d.act_dim,

        // W[act_dim × obs_dim] + b[act_dim] + log_std[act_dim]
        (NetworkKind::Linear, true) => d.obs_dim * d.act_dim + 2 * d.act_dim,

        // W1[H × obs_dim] + b1[H] + W2[act_dim × H] + b2[act_dim]
        // MlpPolicy has a single hidden layer (hidden_dims.len() == 1).
        (NetworkKind::Mlp, false) => {
            let h = d.hidden_dims[0];
            d.obs_dim * h + h + h * d.act_dim + d.act_dim
        }

        // (Mlp, true) is caught by validate() step 4 before reaching here.
        // Falls through to the wildcard arm (returns 0).

        // General multi-layer: sizes = [obs_dim, H1, ..., Hn, act_dim]
        // Sum of (sizes[i] * sizes[i+1] + sizes[i+1]) for each layer.
        (NetworkKind::Autograd, false) => {
            let mut sizes = Vec::with_capacity(d.hidden_dims.len() + 2);
            sizes.push(d.obs_dim);
            sizes.extend_from_slice(&d.hidden_dims);
            sizes.push(d.act_dim);
            sizes.windows(2).map(|w| w[0] * w[1] + w[1]).sum()
        }

        // Same as above + log_std[act_dim]
        (NetworkKind::Autograd, true) => {
            let mut sizes = Vec::with_capacity(d.hidden_dims.len() + 2);
            sizes.push(d.obs_dim);
            sizes.extend_from_slice(&d.hidden_dims);
            sizes.push(d.act_dim);
            let base: usize = sizes.windows(2).map(|w| w[0] * w[1] + w[1]).sum();
            base + d.act_dim
        }

        // Future kinds — can't compute, validation will reject via UnknownKind.
        _ => 0,
    }
}
```

**Note**: The same formula applies to `NetworkDescriptor` for value/Q
network validation. The only difference: `NetworkDescriptor` has no
`stochastic` field, and Q-functions use `obs_dim + act_dim.unwrap()` as
their effective input dimension.

### 7.1.2 Reconstruction helpers (added Phase 2)

`from_checkpoint()` constructors need to reconstruct trait objects from
snapshots. These helpers provide type-safe reconstruction:

**On `PolicyArtifact`** (same match logic as `to_policy()`, tighter return types):
- `to_differentiable_policy() -> Result<Box<dyn DifferentiablePolicy>, ArtifactError>` —
  all 5 policy types implement `DifferentiablePolicy`, so every valid
  `(kind, stochastic)` combo works. Used by REINFORCE, PPO, TD3.
- `to_stochastic_policy() -> Result<Box<dyn StochasticPolicy>, ArtifactError>` —
  only `(Linear, true)` and `(Autograd, true)`. Returns
  `UnsupportedCombination` for non-stochastic descriptors. Used by SAC.

**On `NetworkSnapshot`** (reconstruct critics from checkpoint):
- `to_value_fn() -> Result<Box<dyn ValueFn>, ArtifactError>` — used by PPO
  (`role="value"`).
- `to_q_function() -> Result<Box<dyn QFunction>, ArtifactError>` — used by
  TD3 and SAC. Requires `descriptor.act_dim` to be `Some`.
- `to_differentiable_policy() -> Result<Box<dyn DifferentiablePolicy>, ArtifactError>` —
  used by TD3 for `role="actor_target"`. Requires `descriptor.act_dim` to
  be `Some`.

### 7.2 Provenance assembly patterns

**The algorithm returns a bare artifact. The caller builds provenance.**

The algorithm knows: policy state (descriptor + params).
The caller knows: task name, seed, hyperparams, returned metrics.

**Competition runner pattern:**

```rust
// Inside Competition::run()
let metrics = algorithm.train(&mut env, self.budget, self.seed, &on_epoch);
let artifact = algorithm.policy_artifact().with_provenance(
    TrainingProvenance {
        algorithm: algorithm.name().to_string(),
        task: task.name().to_string(),
        seed: self.seed,
        epochs_trained: metrics.len(),
        final_reward: metrics.last().map_or(0.0, |m| m.mean_reward),
        total_steps: metrics.iter().map(|m| m.total_steps).sum(),
        wall_time_ms: metrics.iter().map(|m| m.wall_time_ms).sum(),
        timestamp: now_iso8601(),
        hyperparams: BTreeMap::new(), // caller populates if desired
        metrics: metrics.clone(),
        parent: None,
    },
);
```

**Curriculum / transfer learning pattern:**

```rust
// Load parent artifact
let parent = PolicyArtifact::load("stage1.artifact.json")?;
let parent_provenance = parent.provenance.clone();
let warm_policy = parent.to_policy()?;

// Train on harder task
let mut td3 = Td3::new(warm_policy, ...);
let metrics = td3.train(&mut hard_env, budget, seed, &on_epoch);

// Build artifact with parent linkage
let artifact = td3.policy_artifact().with_provenance(
    TrainingProvenance {
        algorithm: "TD3".to_string(),
        task: "reaching-6dof-hard".to_string(),
        parent: parent_provenance.map(Box::new),
        // ... other fields ...
        ..Default::default()
    },
);
```

The parent's full provenance chain is preserved — load a 5-stage
curriculum artifact and you can trace back to the original training run.

---

## 8. Integration points

### 8.1 Competition runner

```rust
pub struct RunResult {
    pub task_name: String,
    pub algorithm_name: String,
    pub metrics: Vec<EpochMetrics>,
    pub artifact: PolicyArtifact,       // NEW — includes provenance
}
```

New methods on `CompetitionResult`:

```rust
impl CompetitionResult {
    /// Save all artifacts to a directory.
    /// File naming: {task}_{algorithm}.artifact.json
    pub fn save_artifacts(&self, dir: impl AsRef<Path>) -> Result<(), ArtifactError>;

    /// Best artifact for a task (highest final reward).
    pub fn best_for_task(&self, task: &str) -> Option<&PolicyArtifact>;
}
```

The competition runner builds provenance when constructing `RunResult`
(it has the task, seed, metrics, algorithm name — everything needed).

### 8.2 Bevy replay pattern

The train-then-replay flow in one binary:

```rust
// Phase 1: Train (headless)
let mut cem = Cem::new(policy, hyperparams);
let _metrics = cem.train(&mut env, budget, seed, &on_epoch);
let artifact = cem.policy_artifact();

// Phase 2: Replay (Bevy visualization)
let policy = artifact.to_policy()?;
// Each frame:
let obs = get_observation(&sim);
let action = policy.forward(&obs);
apply_action(&mut sim, &action);
```

No file I/O needed for this pattern. But saving is one line:
`artifact.save("trained.artifact.json")?;`

### 8.3 Curriculum / transfer learning

See §7.2 for the full provenance assembly pattern.

### 8.4 Policy comparison

```rust
let a = PolicyArtifact::load("cem_reaching.artifact.json")?;
let b = PolicyArtifact::load("td3_reaching.artifact.json")?;

let policy_a = a.to_policy()?;
let policy_b = b.to_policy()?;

// Run both on same environment, compare behavior
for obs in test_observations {
    let action_a = policy_a.forward(&obs);
    let action_b = policy_b.forward(&obs);
    // Compare...
}
```

---

## 9. Implementation scope

### Trait changes (breaking) — Done
- `Policy::descriptor()` — 5 concrete impls ✓ Phase 1
- `ValueFn::descriptor()` — 3 concrete impls ✓ Phase 1
- `QFunction::descriptor()` — 3 concrete impls ✓ Phase 1
- `Algorithm::policy_artifact()` — 5 concrete impls ✓ Phase 2
- `Algorithm::checkpoint()` — 5 concrete impls ✓ Phase 2
- `Optimizer::snapshot()` + `load_snapshot()` — 1 concrete impl (Adam) ✓ Phase 2

### Per-algorithm constructors — Done (Phase 2)
- `Cem::from_checkpoint()` + `Reinforce::from_checkpoint()` + `Ppo::from_checkpoint()` + `Td3::from_checkpoint()` + `Sac::from_checkpoint()`

### New types — Done
- `NetworkKind` (shared enum) ✓ Phase 1
- `PolicyDescriptor` ✓ Phase 1
- `NetworkDescriptor` ✓ Phase 1
- `PolicyArtifact` (with `save`, `load`, `to_policy`, `to_differentiable_policy`, `to_stochastic_policy`, `from_policy`, `with_provenance`, `validate`) ✓ Phase 1+2
- `TrainingProvenance` ✓ Phase 1
- `ArtifactError` ✓ Phase 1
- `TrainingCheckpoint` (with `save`, `load`) ✓ Phase 2
- `NetworkSnapshot` (with `to_value_fn`, `to_q_function`, `to_differentiable_policy`) ✓ Phase 2
- `OptimizerSnapshot` ✓ Phase 2

### Serde derives on existing types — Done (Phase 1)
- `EpochMetrics`
- `Activation`
- `OptimizerConfig` (with custom serde for `max_grad_norm` infinity — see §4.7 note)

### Infrastructure changes
- Algorithm struct field additions: promote optimizer instances and mutable
  scalars from `train()` locals to struct fields (§4.9) ✓ Phase 2
- `Cargo.toml`: add `serde` (with `derive` feature), `serde_json` as required deps ✓ Phase 1
- `RunResult` gains `artifact: PolicyArtifact` field — **Phase 3**
- `CompetitionResult` gains `save_artifacts()` and `best_for_task()` — **Phase 3**

### Test mock updates — Done
- `MockQ` (`value.rs` tests): **Done (Phase 1)** — added `obs_dim`,
  `act_dim`, `obs_scale` fields. `descriptor()` returns
  `NetworkKind::Linear`. `MockQ::new(p)` derives dimensions from param count.
- `MockAlgorithm` (`competition.rs` tests): **Done (Phase 2)** — added
  `policy: Box<dyn Policy>` field (a `LinearPolicy::new(1, 1, &[1.0])`).
  Implements `policy_artifact()` and `checkpoint()` with minimal valid data.

### Tests — 362 total (Phase 1: 336, Phase 2: +26)
- Round-trip: create artifact → save → load → validate → to_policy → forward matches ✓
- Every policy type: descriptor correctness ✓
- Every algorithm: policy_artifact produces valid artifact ✓
- Every algorithm: checkpoint round-trip (checkpoint → from_checkpoint → verify params match) ✓
- Every algorithm: checkpoint before train (initial state) ✓
- Optimizer: snapshot → load_snapshot preserves momentum state ✓
- Optimizer: interrupted training matches continuous (momentum continuity) ✓
- Checkpoint/snapshot serde round-trips ✓
- Reconstruction helpers (to_value_fn, to_q_function, to_differentiable/stochastic_policy) ✓
- PolicyDescriptor → NetworkDescriptor conversion ✓
- Validation: version check, param count, obs_scale length, hidden_dims consistency ✓
- Error cases: unsupported version, corrupted JSON, param count mismatch ✓
- Provenance: with_provenance builder, parent linkage ✓
- Competition: save_artifacts, best_for_task — **Phase 3**

---

## 10. What this enables — the timeline

### Now (ships with this spec)
- Competition tests save winning policies for inspection
- Train-then-replay examples (one binary: train inline → Bevy visualization)
- Compare policies across experiments
- Resume training from checkpoint (full Tier 2 — all 5 algorithms)
- Epoch-level checkpointing (call `checkpoint()` or `policy_artifact()` from `on_epoch`)
- Curriculum learning (load easy-task policy → continue on hard task)

### Near (enabled by this foundation, built when needed)
- Policy evaluation harness (load saved policy, evaluate on N episodes)
- Architecture search (try different hidden_dims, save best artifact)
- Policy distillation (train small policy to mimic large one)
- Training analysis dashboards (load provenance, plot curves)

### Medium
- Policy zoo / library (directory of artifacts with metadata)
- Replay buffer checkpointing (Tier 3 — large, needs real-world sizing data)

### Far
- Export to deployment formats (ONNX, TFLite — built on top of artifacts)
- Cloud training → local deployment pipeline
- Multi-agent (each agent has its own artifact)
- Policy versioning (git for artifacts — provenance chain is the history)
- Hierarchical policies (artifact per sub-policy)
- Online learning (periodic artifact snapshots as policy evolves in deployment)

---

## 11. Phasing

**Phase 1 — Foundation types + descriptor trait surgery** ✓ `066abea`
- All types in `src/artifact.rs` (ArtifactError co-located, not in `error.rs`)
- 11 descriptor impls (5 Policy + 3 ValueFn + 3 QFunction)
- 28 tests (descriptors, round-trips, validation, provenance, serde format)
- MockQ updated with descriptor fields (Phase 2 still needs MockAlgorithm)

**Phase 2 — Algorithm integration + checkpointing** ✓ `dee449b`
- State promotion (§4.9): optimizers + mutable scalars moved to struct fields
  for all 5 algorithms (see §4.9 implementation notes for borrow workaround)
- New types: TrainingCheckpoint, NetworkSnapshot, OptimizerSnapshot
- TrainingCheckpoint save/load with version check
- Optimizer::snapshot() + load_snapshot() on trait + Adam impl
  (see §4.7 implementation note for infinity serde)
- Algorithm::policy_artifact() + checkpoint() on trait + 5 real impls
- Reconstruction helpers: PolicyArtifact (to_differentiable_policy,
  to_stochastic_policy) + NetworkSnapshot (to_value_fn, to_q_function,
  to_differentiable_policy) — see §7.1.2
- PolicyDescriptor → NetworkDescriptor From impl (for TD3 target_policy)
- Per-algorithm from_checkpoint() constructors (5 impls)
- MockAlgorithm updated with LinearPolicy field + trait impls
- 26 new tests (362 total), clippy clean

**Phase 3 — Competition integration**
- RunResult gains `artifact: PolicyArtifact` field (with provenance)
- Competition runner builds provenance during `run()` — it has the task,
  seed, metrics, algorithm name, wall time (everything for TrainingProvenance)
- `algorithm.policy_artifact()` returns a bare artifact; `run()` attaches
  provenance via `.with_provenance()` before storing on RunResult
- CompetitionResult gains `save_artifacts()`, `best_for_task()`
- Competition tests verify artifacts

**Phase 3 implementation notes**:
- `Competition::run()` is in `competition.rs:216-263`. It currently pushes
  `RunResult { task_name, algorithm_name, metrics }`. The change: call
  `algorithm.policy_artifact()` after `train()`, attach provenance, add to
  RunResult. The `algorithm` variable is `Box<dyn Algorithm>` — it has
  `policy_artifact()` from the Phase 2 trait extension.
- `RunResult` struct is at `competition.rs:19-26`. Adding `artifact` field
  breaks the construction site in `run()` (line 254) and any test code
  that constructs RunResult directly — grep for `RunResult {`.
- `RunResult` derives `Clone` — `PolicyArtifact` also derives `Clone`, so
  no issue.
- MockAlgorithm already implements `policy_artifact()` (Phase 2) — its
  artifact will have a tiny 1×1 LinearPolicy, which is fine for tests.
- TrainingCheckpoint save/load is already done (Phase 2). Phase 3 only
  needs PolicyArtifact save/load, which is also done (Phase 1).

**Phase 4 — Visual proof**
- Train-then-replay Bevy example (CEM on reaching-2dof)
- Trains inline (~20 epochs), switches to Bevy visualization
- Saves artifact to disk as side effect
- This is the user-facing proof that the system works

---

## 12. Spec grading rubric

Implementation begins only when every criterion reaches A+.
Each checkbox is pass/fail — verified by reading code, running grep,
or reasoning from the spec. A criterion gets A+ when every box is
checked. Any unchecked box means the spec has a gap to fix first.

### Criterion 1: Codebase verification
> Every claim about existing code is verified against the actual source.
> Merge of accuracy + stress testing — you can't grade one without the other.

**Trait impl counts** (grep `impl Trait for` in `sim/L0/ml-bridge/src/`):
- [ ] `impl Policy for` — confirm exactly 5 types, list them, verify spec matches
- [ ] `impl DifferentiablePolicy for` — list all, note which are also Policy impls
- [ ] `impl StochasticPolicy for` — list all
- [ ] `impl ValueFn for` — confirm exactly 3 types, list them
- [ ] `impl QFunction for` — confirm exactly 3 types, list them
- [ ] `impl Algorithm for` — confirm exactly 5 types, list them (include test mocks)
- [ ] `impl Optimizer for` — confirm exactly 1 type (Adam)

**Struct fields** (read each concrete type's struct definition):
- [ ] For each policy type: verify obs_dim, act_dim, obs_scale are stored fields or derivable (AutogradPolicy derives obs_dim from obs_scale.len() — see §5.1)
- [ ] For MlpPolicy: verify hidden_dim is a stored field (note: single usize, not Vec)
- [ ] For AutogradPolicy: verify hidden layer info is stored (Vec<LayerOffsets>, not Vec<usize> — descriptor() derives hidden_dims from offsets — see §5.1)
- [ ] For each policy type: verify activation is stored or implicit (LinearPolicy has no activation field; MlpPolicy has no activation field, hardcodes Tanh — see §5.1)
- [ ] For Adam: verify m, v, t are stored fields
- [ ] For each algorithm: list every Box<dyn Policy/DifferentiablePolicy/StochasticPolicy> field (note: TD3 uses DifferentiablePolicy, SAC uses StochasticPolicy — see §5.4)
- [ ] For each algorithm: list every Box<dyn ValueFn> field
- [ ] For each algorithm: list every Box<dyn QFunction> field
- [ ] For each algorithm: verify optimizer instances are struct fields (requires state promotion per §4.9 — currently OptimizerConfig.build() inline)
- [ ] For TD3: verify target networks are separate fields, count them (3: target_policy, target_q1, target_q2)
- [ ] For SAC: verify target networks are separate fields, count them (2: target_q1, target_q2 — no target policy)
- [ ] For REINFORCE: verify sigma is a stored field (requires state promotion per §4.9 — currently train()-local)
- [ ] For SAC: verify log_alpha is a stored field (requires state promotion per §4.9 — currently train()-local)
- [ ] For CEM: verify noise_std is a stored field (requires state promotion per §4.9 — currently train()-local)

**Existing types and traits** (read the actual trait definitions):
- [ ] Policy trait: list every method, confirm spec's "existing methods unchanged" is accurate
- [ ] ValueFn trait: list every method
- [ ] QFunction trait: list every method
- [ ] Algorithm trait: list every method
- [ ] Optimizer trait: list every method
- [ ] EpochMetrics: list every field, confirm spec's TrainingProvenance usage matches
- [ ] OptimizerConfig: list every variant and its fields
- [ ] RunResult: list every field (before our changes)
- [ ] CompetitionResult: list every method
- [ ] Competition::run() flow: verify it matches §7.2 provenance assembly pattern
- [ ] Activation enum: list variants, confirm serde compatibility

**Grade**: ___

### Criterion 2: Type system soundness
> Every proposed type, trait method, and impl is valid Rust.

**Object safety** (trait methods must work with `Box<dyn Trait>`):
- [ ] `Policy::descriptor(&self) -> PolicyDescriptor` — no Self, no generics
- [ ] `ValueFn::descriptor(&self) -> NetworkDescriptor` — no Self, no generics
- [ ] `QFunction::descriptor(&self) -> NetworkDescriptor` — no Self, no generics
- [ ] `Algorithm::policy_artifact(&self) -> PolicyArtifact` — no Self, no generics
- [ ] `Algorithm::checkpoint(&self) -> TrainingCheckpoint` — no Self, no generics
- [ ] `Optimizer::snapshot(&self, role: &str) -> OptimizerSnapshot` — no Self, no generics
- [ ] `Optimizer::load_snapshot(&mut self, snapshot: &OptimizerSnapshot)` — no Self, no generics

**Serde compatibility**:
- [ ] `EpochMetrics` — all fields are serde-friendly (BTreeMap<String, f64> works)
- [ ] `OptimizerConfig` — enum with named fields serializes correctly via serde
- [ ] `Activation` — simple enum, serde derives don't conflict with existing derives
- [ ] `TrainingProvenance.parent: Option<Box<TrainingProvenance>>` — recursive type compiles with serde
- [ ] All new types: no fields that block `Debug, Clone, Serialize, Deserialize`
- [ ] `PolicyArtifact` stores `Vec<f64>` not `Box<dyn Policy>` (dyn Trait can't derive Serialize)
- [ ] `#[non_exhaustive]` on NetworkKind — match with `_ =>` wildcard compiles, serde deserializes unknown variants as error (not panic)

**NaN / Infinity**:
- [ ] `serde_json` rejects NaN and Infinity f64 by default — addressed in §6.6 (finite-value policy)
- [ ] EpochMetrics.mean_reward can be NaN — validate() catches non-finite values before save() (§6.6)
- [ ] 0-epoch provenance uses 0.0 not f64::NAN (§7.2)
- [ ] Spec specifies: validate all f64 are finite before serialization, reject with ArtifactError::NonFiniteValue (§4.6, §6.6, §7.1)

**Grade**: ___

### Criterion 3: Completeness
> No gaps — every path through the system is specified.

**Descriptor ↔ reconstruction mapping**:
- [ ] Every concrete Policy type can produce a valid PolicyDescriptor
- [ ] Every valid (kind, stochastic) combination maps to a concrete type OR a documented ArtifactError (§7.1 lists all 7 match arms)
- [ ] Specifically: (Mlp, stochastic: true) — returns ArtifactError::UnsupportedCombination (§4.6, §7.1)
- [ ] Specifically: (Linear, stochastic: true) — LinearStochasticPolicy exists. Verify.
- [ ] Specifically: (Autograd, stochastic: true) — AutogradStochasticPolicy exists. Verify.
- [ ] Every concrete ValueFn type can produce a valid NetworkDescriptor
- [ ] Every concrete QFunction type can produce a valid NetworkDescriptor

**Param count formula**:
- [ ] `compute_param_count()` formula is defined in §7.1.1 with full Rust implementation
- [ ] Formula covers Linear (non-stochastic): `obs_dim * act_dim + act_dim`
- [ ] Formula covers Linear (stochastic): `obs_dim * act_dim + 2 * act_dim`
- [ ] Formula covers Mlp (non-stochastic, single hidden layer): `obs_dim * H + H + H * act_dim + act_dim`
- [ ] Formula covers Autograd (non-stochastic, arbitrary hidden layers): windowed sum
- [ ] Formula covers Autograd (stochastic, arbitrary hidden layers): windowed sum + act_dim
- [ ] Each formula verified against actual n_params() output from constructing a test policy

**Checkpoint completeness**:
- [ ] Every algorithm's checkpoint() captures ALL mutable state — requires state promotion (§4.9) so optimizers and scalars are on &self
- [ ] Every algorithm has a from_checkpoint() that restores ALL captured state
- [ ] Checkpoint table (§4.7) has correct network count per algorithm (TD3 target_policy is in critics Vec as "actor_target")
- [ ] Checkpoint table has correct optimizer count per algorithm (verified against struct fields)
- [ ] SAC optimizer count verified: 3 (actor, q1, q2) — confirmed in §4.7 table and §11 Phase 2

**Edge cases**:
- [ ] checkpoint() before train() — returns initial-state checkpoint (optimizers at t=0, scalars at init values per §4.9)
- [ ] policy_artifact() before train() — what happens? Returns initial random weights? Specified.
- [ ] Loading artifact with mismatched version — ArtifactError::UnsupportedVersion
- [ ] Loading artifact with unknown NetworkKind — ArtifactError::UnknownKind
- [ ] Loading artifact with wrong param count — ArtifactError::ParamCountMismatch
- [ ] Loading artifact with hidden_dims mismatch (Linear + hidden_dims=[64]) — ArtifactError::LinearHiddenDims
- [ ] Loading corrupted JSON — ArtifactError::Json
- [ ] Provenance chain depth: 5-stage curriculum — serializes without stack overflow

**Grade**: ___

### Criterion 4: Breaking change audit
> Every existing caller that breaks is identified with a migration path.

**Trait impls that need new methods**:
- [ ] List every `impl Policy for X` (including test mocks outside ml-bridge src/)
- [ ] List every `impl ValueFn for X` (including test mocks)
- [ ] List every `impl QFunction for X` (including MockQ in value.rs tests — needs descriptor fields, see §9 test mock updates)
- [ ] List every `impl Algorithm for X` (including MockAlgorithm in competition.rs tests — needs policy field, see §9 test mock updates)
- [ ] List every `impl Optimizer for X`
- [ ] For each, confirm the new method can be implemented (the type has the data needed — mocks need updates per §9)

**Struct changes**:
- [ ] RunResult — list every place it's constructed (competition.rs)
- [ ] RunResult — list every place it's destructured or field-accessed (competition tests, any examples)
- [ ] EpochMetrics — adding Serialize/Deserialize derives has no side effects on existing code
- [ ] Activation — adding Serialize/Deserialize derives has no side effects
- [ ] OptimizerConfig — adding Serialize/Deserialize derives has no side effects

**Downstream consumers**:
- [ ] Bevy examples that use Policy trait — list them, note they need descriptor()
- [ ] Bevy examples that use Algorithm trait — list them, note they need policy_artifact() + checkpoint()
- [ ] Competition tests — list every test function, note which need changes and what changes
- [ ] Benchmarks — check if any benchmark code impls these traits

**Dependency additions**:
- [ ] `serde` + `serde_json` — check for workspace version conflicts in root Cargo.toml
- [ ] `serde` derive feature — confirm workspace supports it or add it

**Grade**: ___

### Criterion 5: Internal consistency
> No contradictions between sections. Types match their usage everywhere.

- [ ] §4 type definitions match §5 trait signatures (field names, types, method signatures)
- [ ] §5 trait method counts match §9 implementation scope counts
- [ ] §7.1 to_policy() match arms cover every (kind, stochastic) combo — 5 valid + 1 error (Mlp/true) + wildcard (unknown)
- [ ] §7.1 validate() checks reference every ArtifactError variant from §4.6 (including NonFiniteValue and UnsupportedCombination)
- [ ] §7.2 provenance assembly pattern uses TrainingProvenance fields exactly as defined in §4.5
- [ ] §6.4 JSON example field names match §4 struct field names exactly
- [ ] §6.4 JSON example nesting matches struct nesting (descriptor inside artifact, metrics inside provenance)
- [ ] §8.1 RunResult changes match §9 infrastructure changes
- [ ] §11 phasing doesn't reference types before the phase that defines them
- [ ] §4.7 checkpoint table matches §11 Phase 2 checkpoint impl list (SAC = 3 optimizers in both)
- [ ] §4.6 ArtifactError variants cover every error return in §7.1 AND §4.8 (10 variants total)

**Grade**: ___

### Criterion 6: Robustness
> Edge cases, serialization limits, and real-world scenarios.

**Serialization edge cases**:
- [ ] NaN/Infinity handling — §6.6 finite-value policy: validate() rejects, save() calls validate(), ArtifactError::NonFiniteValue
- [ ] Empty params Vec (0-param policy) — round-trips correctly (JSON `[]`)
- [ ] Empty metrics Vec (0-epoch training) — provenance handles it (JSON `[]`, final_reward = 0.0)
- [ ] Empty obs_scale Vec (obs_dim = 0) — validate() passes if obs_dim == 0 and obs_scale == [] (both are 0)
- [ ] Very long provenance chain (depth 100) — serde_json handles recursive Box without stack overflow
- [ ] Unicode in task/algorithm names — serde_json handles it (JSON is UTF-8)
- [ ] f64 precision: serde_json uses `ryu` for formatting — preserves enough digits for lossless round-trip of normal f64 values

**Scale**:
- [ ] 20K-epoch training run: estimate provenance metrics Vec size in bytes. Is it acceptable in a single JSON file?
- [ ] Million-param policy: estimate JSON artifact file size. Confirm binary escape hatch is sufficient.
- [ ] Checkpoint frequency: calling checkpoint() every 100 epochs on TD3 (6 networks) — estimate per-call allocation cost

**Backward/forward compatibility**:
- [ ] Spec explicitly states: no `#[serde(deny_unknown_fields)]` — hard rule in §6.3
- [ ] All Option<T> fields use `#[serde(default)]` — hard rule in §6.3
- [ ] Adding a new optional field to PolicyArtifact in future v2 — old code (v1) can still read it (serde ignores unknown fields)
- [ ] Loading a v1 artifact with v2 code that added an optional field — missing field deserializes as None/default via #[serde(default)]

**Grade**: ___

### Criterion 7: Future-proofing
> Extensibility verified by concrete grep-based analysis, not thought experiments.

**Adding a new NetworkKind (e.g., Cnn)**:
- [ ] grep every usage of `NetworkKind` — list all match arms and construction sites
- [ ] Confirm changes are: 1 enum variant + 1 match arm in to_policy() + 1 descriptor impl + 1 param count formula case
- [ ] Confirm `#[non_exhaustive]` means existing serialized artifacts still load (unknown variant = error, not panic)

**Adding a new Algorithm (e.g., A2C)**:
- [ ] grep every usage of `Algorithm` trait — list all places that iterate/dispatch over algorithms
- [ ] Confirm changes are: impl Algorithm (3 methods) + from_checkpoint() constructor
- [ ] No changes to existing algorithms, types, or infrastructure

**Adding a new Optimizer (e.g., AdamW)**:
- [ ] grep every usage of `Optimizer` trait — list all call sites
- [ ] Confirm changes are: impl Optimizer (existing methods + snapshot + load_snapshot) + OptimizerConfig variant
- [ ] No changes to existing optimizer or algorithm code

**Format evolution**:
- [ ] JSON → binary: confirm save/load are the only methods that touch the format — types, traits, validation are format-agnostic
- [ ] Adding replay buffer to checkpoint: confirm it's an additive field, no structural changes to TrainingCheckpoint

**Grade**: ___

### Overall

| Criterion | Grade | Notes |
|-----------|-------|-------|
| 1. Codebase verification | | |
| 2. Type system soundness | | |
| 3. Completeness | | |
| 4. Breaking change audit | | |
| 5. Internal consistency | | |
| 6. Robustness | | |
| 7. Future-proofing | | |

**Implementation gate**: All 7 criteria must be A+ before Phase 1 begins.

When grading reveals a spec gap, the fix goes into the spec first (new
section or updated section), then the checkbox gets re-evaluated. The
rubric grades the spec — it doesn't grade the code.
