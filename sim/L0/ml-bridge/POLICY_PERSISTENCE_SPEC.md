# Policy Persistence — Artifact & Checkpoint Spec

> Build foundational infrastructure for saving, loading, and transferring
> learned behavior across the ML domain. Not a bolt-on — a structural part
> of every trait, every algorithm, every policy type. The currency of the
> ML layer.

**Status**: v2 — Open questions resolved, ready for implementation review
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
| **No dead code on traits** | Every trait method must have a real implementation. No `todo!()` bodies, no panic stubs. If we aren't implementing it yet, the method doesn't go on the trait yet. Define the types now; add the trait methods when the first algorithm implements them. |

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

**Scope of this spec**: Tier 1 fully implemented. Tier 2 types defined,
trait methods and implementations deferred until first real use. Tier 3
types defined, everything else deferred.

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

    #[error(transparent)]
    Io(#[from] std::io::Error),

    #[error(transparent)]
    Json(#[from] serde_json::Error),
}
```

Used by `save()`, `load()`, `to_policy()`, and `validate()`.

### 4.7 TrainingCheckpoint — the resume point (types only, no trait methods)

These types are defined now for forward compatibility. Trait methods
(`Algorithm::checkpoint()`, `Optimizer::snapshot()`) are added later
when the first algorithm actually implements resume-from-checkpoint.
No `todo!()` stubs.

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingCheckpoint {
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
returning a struct literal from fields already stored on the struct.

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

### 5.4 Algorithm trait — add `policy_artifact()` only

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
}
```

**No `checkpoint()` method.** The `TrainingCheckpoint` type exists (§4.7)
but the trait method is added later when the first algorithm implements
resume-from-checkpoint. This avoids `todo!()` stubs.

`train()` return type is **unchanged** (`Vec<EpochMetrics>`). The artifact
is accessed via the separate `policy_artifact()` method — more flexible
(callable any time) and non-breaking for existing competition assertions.

### 5.5 Optimizer, ValueFn, QFunction — no new trait methods yet

`Optimizer::snapshot()` / `load_snapshot()` are deferred until checkpoint
implementations begin. The `OptimizerSnapshot` type exists (§4.7) but
no trait method forces a panic stub.

`ValueFn::descriptor()` and `QFunction::descriptor()` are added in Phase 3
when checkpoint types need them. Not needed for Tier 1 artifacts.

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

### 6.5 Future binary format (not this spec)

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
        // Match on (kind, stochastic) to select constructor.
        // Call set_params() to load weights.
        // Unknown kind (from future version) → ArtifactError::UnknownKind
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

    /// Save to JSON file.
    pub fn save(&self, path: impl AsRef<Path>) -> Result<(), ArtifactError> {
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
        // 4. param count (compute expected from descriptor, compare to actual)
        let expected = compute_param_count(&self.descriptor);
        if self.params.len() != expected {
            return Err(ArtifactError::ParamCountMismatch {
                expected,
                actual: self.params.len(),
            });
        }
        Ok(())
    }
}
```

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
        final_reward: metrics.last().map_or(f64::NAN, |m| m.mean_reward),
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

### Trait changes (breaking)
- `Policy::descriptor()` — 5 concrete impls
- `Algorithm::policy_artifact()` — 5 concrete impls

### New types
- `NetworkKind` (shared enum)
- `PolicyDescriptor`
- `NetworkDescriptor` (defined now, used by checkpoint types)
- `PolicyArtifact` (with `save`, `load`, `to_policy`, `from_policy`, `with_provenance`, `validate`)
- `TrainingProvenance`
- `ArtifactError`
- `TrainingCheckpoint`, `NetworkSnapshot`, `OptimizerSnapshot` (types only, no trait methods)

### Serde derives on existing types
- `EpochMetrics`
- `Activation`
- `OptimizerConfig`

### Infrastructure changes
- `RunResult` gains `artifact: PolicyArtifact` field
- `CompetitionResult` gains `save_artifacts()` and `best_for_task()`
- `Cargo.toml`: add `serde` (with `derive` feature), `serde_json` as required deps

### Deferred trait methods (added when first implementation exists)
- `ValueFn::descriptor()` — 3 impls (when checkpoint implementations begin)
- `QFunction::descriptor()` — 3 impls (when checkpoint implementations begin)
- `Algorithm::checkpoint()` — 5 impls (per-algorithm, as needed)
- `Optimizer::snapshot()` + `load_snapshot()` — 1 impl (Adam)

### New tests
- Round-trip: create artifact → save → load → validate → to_policy → forward matches
- Every policy type: descriptor correctness
- Every algorithm: policy_artifact produces valid artifact after training
- Validation: version check, param count, obs_scale length, hidden_dims consistency
- Error cases: unsupported version, corrupted JSON, param count mismatch
- Provenance: with_provenance builder, parent linkage
- Competition: save_artifacts writes correct files, best_for_task returns highest reward

---

## 10. What this enables — the timeline

### Now
- Competition tests save winning policies for inspection
- Train-then-replay examples (one binary: train inline → Bevy visualization)
- Compare policies across experiments

### Near
- Epoch-level checkpointing (call `policy_artifact()` from `on_epoch`)
- Curriculum learning (load easy-task policy → continue on hard task)
- Policy evaluation harness (load saved policy, evaluate on N episodes)
- Architecture search (try different hidden_dims, save best artifact)

### Medium
- Resume training from checkpoint (Tier 2 trait methods + implementations)
- Policy distillation (train small policy to mimic large one)
- Policy zoo / library (directory of artifacts with metadata)
- Training analysis dashboards (load provenance, plot curves)

### Far
- Export to deployment formats (ONNX, TFLite — built on top of artifacts)
- Cloud training → local deployment pipeline
- Multi-agent (each agent has its own artifact)
- Policy versioning (git for artifacts — provenance chain is the history)
- Hierarchical policies (artifact per sub-policy)
- Online learning (periodic artifact snapshots as policy evolves in deployment)

---

## 11. Phasing

**Phase 1 — Foundation types + Policy trait surgery**
- New deps: `serde` (with `derive`), `serde_json`
- Serde derives on EpochMetrics, Activation, OptimizerConfig
- New types: NetworkKind, PolicyDescriptor, PolicyArtifact, TrainingProvenance, ArtifactError
- Policy::descriptor() on trait + 5 impls
- PolicyArtifact: from_policy(), with_provenance(), save(), load(), to_policy(), validate()
- Round-trip tests for every policy type
- Validation tests (version, param count, obs_scale, hidden_dims, error cases)

**Phase 2 — Algorithm integration**
- Algorithm::policy_artifact() on trait + 5 impls
- RunResult gains artifact field
- Competition runner builds provenance during run()
- CompetitionResult gains save_artifacts(), best_for_task()
- Competition tests verify artifacts

**Phase 3 — Visual proof**
- Train-then-replay Bevy example (CEM on reaching-2dof)
- Trains inline (~20 epochs), switches to Bevy visualization
- Saves artifact to disk as side effect
- This is the user-facing proof that the system works

**Phase 4 — Critic descriptors + checkpoint types (when needed)**
- NetworkDescriptor already defined in Phase 1
- ValueFn::descriptor() + 3 impls
- QFunction::descriptor() + 3 impls
- TrainingCheckpoint, NetworkSnapshot, OptimizerSnapshot types already defined
- Algorithm::checkpoint() added to trait when first algorithm implements it
- Optimizer::snapshot() + load_snapshot() added to trait when needed

**Phase 5 — Checkpoint implementations (per-algorithm, as needed)**
- CEM checkpoint (simple: policy + noise_std)
- REINFORCE checkpoint (policy + optimizer + sigma)
- PPO checkpoint (policy + value + optimizers)
- TD3 checkpoint (policy + targets + Q1/Q2 + targets + optimizers)
- SAC checkpoint (full state)
