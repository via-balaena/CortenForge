# Policy Persistence — Artifact & Checkpoint Spec

> Build foundational infrastructure for saving, loading, and transferring
> learned behavior across the ML domain. Not a bolt-on — a structural part
> of every trait, every algorithm, every policy type. The currency of the
> ML layer.

**Status**: v1 — Design phase (open questions marked with **OPEN**)
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

**Scope of this spec**: Tier 1 fully implemented. Tier 2 interfaces defined,
implementations phased. Tier 3 interfaces defined, implementations deferred.

---

## 4. Core types

### 4.1 PolicyDescriptor — the recipe card

Enough information to reconstruct an empty policy with the same structure.

```rust
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PolicyDescriptor {
    /// Which concrete type to construct.
    pub kind: PolicyKind,
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

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum PolicyKind {
    Linear,
    Mlp,
    Autograd,
}
```

`#[non_exhaustive]` on `PolicyKind` — future kinds (Cnn, Rnn, Transformer,
MixtureOfExperts) can be added without breaking existing serialized files
or match arms.

### 4.2 NetworkDescriptor — recipe card for critics

Same pattern as PolicyDescriptor but for ValueFn and QFunction types.

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

```rust
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
pub enum NetworkKind {
    Linear,
    Mlp,
    Autograd,
}
```

**OPEN 1: Should PolicyDescriptor and NetworkDescriptor be the same type?**

Arguments for separate: policies have `stochastic` flag, critics don't. Policies
are the deployable star; critics are supporting cast. Semantic clarity.

Arguments for unified: fields overlap 90%. One fewer type to maintain. Unified
reconstruction logic.

Current lean: **Separate.** The `stochastic` flag is policy-specific and would be
awkward on a critic descriptor. Semantic clarity > fewer types.

### 4.3 PolicyArtifact — the deployable brain

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

### 4.4 TrainingProvenance — the lineage

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

### 4.5 TrainingCheckpoint — the resume point

**OPEN 2: Define `checkpoint()` on Algorithm trait now, or defer?**

If now: every algorithm must implement it (can `todo!()` the bodies).
The trait signature is the expensive thing to change later.

If defer: define the types but don't add the trait method yet. Add it
when the first algorithm actually needs cross-session resume.

Current lean: **Define the trait method now with `todo!()` bodies.** The types:

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
    pub config: OptimizerConfigSnapshot,
    /// First moment estimates (Adam m).
    pub m: Vec<f64>,
    /// Second moment estimates (Adam v).
    pub v: Vec<f64>,
    /// Step count (Adam t).
    pub t: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct OptimizerConfigSnapshot {
    pub kind: String,       // "Adam"
    pub lr: f64,
    pub beta1: f64,
    pub beta2: f64,
    pub eps: f64,
    pub max_grad_norm: f64,
}
```

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
    /// Can be called after train() for the final policy, or between
    /// training sessions for intermediate snapshots.
    fn policy_artifact(&self) -> PolicyArtifact;

    /// Extract full training state for later resumption.
    ///
    /// Includes policy, critics, optimizer momentum — everything needed
    /// to continue training without regression.
    fn checkpoint(&self) -> TrainingCheckpoint;
}
```

**OPEN 3: Should `train()` return type change to bundle metrics + artifact?**

Option A (current lean): Keep `Vec<EpochMetrics>`. Add `policy_artifact()` separately.
- Less breaking. More flexible (call any time, not just after train).
- Competition tests only assert on metrics — no code change needed in assertions.

Option B: Return `TrainingResult { metrics: Vec<EpochMetrics>, artifact: PolicyArtifact }`.
- Can't forget to extract. More "foundational" — artifact is a result, not a query.
- Larger break. Every caller must destructure.

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

1 implementation: Adam. (OptimizerConfig is an enum — future optimizers
like AdamW would also implement this.)

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
- `TrainingBudget` (for checkpoint metadata)
- `OptimizerConfig` (for checkpoint metadata)

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
- Readers accept `version <= CURRENT_VERSION`
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

## 7. Reconstruction

```rust
impl PolicyArtifact {
    /// Reconstruct a live policy from this artifact.
    /// The returned policy is ready for forward() — inference/replay only.
    pub fn into_policy(&self) -> Box<dyn Policy> {
        // Match on (kind, stochastic) to select constructor.
        // Call set_params() to load weights.
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

    /// Save to JSON file.
    pub fn save(&self, path: impl AsRef<Path>) -> io::Result<()>;

    /// Load from JSON file.
    pub fn load(path: impl AsRef<Path>) -> io::Result<Self>;

    /// Validate that params length matches descriptor's implied param count.
    pub fn validate(&self) -> Result<(), String>;
}
```

`from_policy()` is critical — Bevy examples don't go through
`Algorithm::train()`, but they need to produce artifacts too. This bridges
the gap: any `&dyn Policy` becomes a `PolicyArtifact` in one call.

---

## 8. Integration points

### 8.1 Competition runner

```rust
pub struct RunResult {
    pub task_name: String,
    pub algorithm_name: String,
    pub metrics: Vec<EpochMetrics>,
    pub artifact: PolicyArtifact,       // NEW
}
```

New methods on `CompetitionResult`:

```rust
impl CompetitionResult {
    /// Save all artifacts to a directory.
    /// File naming: {task}_{algorithm}.artifact.json
    pub fn save_artifacts(&self, dir: impl AsRef<Path>) -> io::Result<()>;

    /// Best artifact for a task (highest final reward).
    pub fn best_for_task(&self, task: &str) -> Option<&PolicyArtifact>;
}
```

After this change, every competition run automatically produces saveable
artifacts. The `save_artifacts()` call is optional — but the artifacts
are always there in `RunResult`.

### 8.2 Bevy replay pattern

The train-then-replay flow in one binary:

```rust
// Phase 1: Train (headless)
let mut cem = Cem::new(policy, hyperparams);
let _metrics = cem.train(&mut env, budget, seed, &on_epoch);
let artifact = cem.policy_artifact();

// Phase 2: Replay (Bevy visualization)
let policy = artifact.into_policy();
// Each frame:
let obs = get_observation(&sim);
let action = policy.forward(&obs);
apply_action(&mut sim, &action);
```

No file I/O needed for this pattern. But saving is one line:
`artifact.save("trained.artifact.json")?;`

### 8.3 Curriculum / transfer learning

```rust
// Stage 1: Train on easy task
let artifact_easy = cem.policy_artifact();
artifact_easy.save("stage1.artifact.json")?;

// Stage 2: Load and continue on hard task
let loaded = PolicyArtifact::load("stage1.artifact.json")?;
let mut warm_policy = loaded.into_policy();
// Pass warm_policy to a new algorithm — weights are pre-trained
let mut td3 = Td3::new(warm_policy, ...);
let _metrics = td3.train(&mut hard_env, budget, seed, &on_epoch);
let artifact_hard = td3.policy_artifact();
// artifact_hard.provenance.parent == loaded.provenance
```

### 8.4 Policy comparison

```rust
let a = PolicyArtifact::load("cem_reaching.artifact.json")?;
let b = PolicyArtifact::load("td3_reaching.artifact.json")?;

let policy_a = a.into_policy();
let policy_b = b.into_policy();

// Run both on same environment, compare behavior
for obs in test_observations {
    let action_a = policy_a.forward(&obs);
    let action_b = policy_b.forward(&obs);
    // Compare...
}
```

---

## 9. Open questions

### OPEN 1: Should PolicyDescriptor and NetworkDescriptor be unified?

**Separate (current lean):** Policies have `stochastic` flag that critics
don't. Semantic clarity. Policies are the deployable output; critics are
training infrastructure.

**Unified:** 90% field overlap. One fewer type. Single reconstruction path.

### OPEN 2: Define `checkpoint()` on Algorithm trait now, or defer?

**Now (current lean):** Define the trait method. `todo!()` the bodies.
The signature is the hard-to-change part. Implementations are easy to
fill in one algorithm at a time.

**Defer:** Define the types but no trait method. Add method later when
first algorithm actually needs cross-session resume.

### OPEN 3: Should `train()` return type change?

**Keep `Vec<EpochMetrics>` + separate `policy_artifact()` (current lean):**
Less breaking. More flexible. Competition assertions unchanged.

**Return `TrainingResult` bundling both:** Can't forget to extract.
More "foundational". Larger break.

### OPEN 4: EpochMetrics — derive Serialize directly, or mirror type?

**Derive directly (current lean):** serde is a required dep, not optional.
No ceremony, no conversion.

**Mirror type:** `SerializableEpochMetrics` with `From<EpochMetrics>`.
Decouples public API from serde. Adds boilerplate.

### OPEN 5: Replay buffer in checkpoint type?

**Optional field, implement later (current lean):** Define
`replay_buffer: Option<ReplayBufferSnapshot>` in `TrainingCheckpoint`.
Always serialize as `None` for now. Fill in when cross-session off-policy
training is needed.

**Omit entirely:** Don't even define the field. Add when needed. Cleaner
now but a structural change to the checkpoint format later.

### OPEN 6: File naming convention?

Options for auto-saved competition artifacts:
- `{task}_{algorithm}.artifact.json` (simple, no seed — overwrite on rerun)
- `{task}_{algorithm}_{seed}.artifact.json` (reproducibility)
- `{task}_{algorithm}_{seed}_{timestamp}.artifact.json` (never overwrite)

---

## 10. Implementation scope

### Trait changes (breaking)
- `Policy::descriptor()` — 5 concrete impls
- `ValueFn::descriptor()` — 3 concrete impls
- `QFunction::descriptor()` — 3 concrete impls
- `Algorithm::policy_artifact()` — 5 concrete impls
- `Algorithm::checkpoint()` — 5 concrete impls (todo!() initially)
- `Optimizer::snapshot()` + `load_snapshot()` — 1 concrete impl (Adam)

### New types
- `PolicyDescriptor`, `PolicyKind`
- `NetworkDescriptor`, `NetworkKind`
- `PolicyArtifact` (with `save`, `load`, `into_policy`, `from_policy`, `validate`)
- `TrainingProvenance`
- `TrainingCheckpoint`, `NetworkSnapshot`, `OptimizerSnapshot`, `OptimizerConfigSnapshot`

### Serde derives on existing types
- `EpochMetrics`
- `Activation`
- `TrainingBudget`
- `OptimizerConfig`

### Infrastructure changes
- `RunResult` gains `artifact: PolicyArtifact` field
- `CompetitionResult` gains `save_artifacts()` and `best_for_task()`
- `Cargo.toml`: add `serde`, `serde_json` as required deps

### New tests
- Round-trip: create artifact → save → load → validate → into_policy → forward matches
- Every policy type: descriptor round-trip
- Every algorithm: policy_artifact produces valid artifact after training
- Provenance chain: parent linkage for curriculum learning
- Competition: save_artifacts writes correct files

---

## 11. What this enables — the timeline

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
- Resume training from checkpoint (full Tier 2 implementation)
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

## 12. Phasing (proposed)

**Phase 1 — Foundation types + Policy trait surgery**
- New types: PolicyDescriptor, PolicyKind, PolicyArtifact, TrainingProvenance
- Serde derives on EpochMetrics, Activation
- Policy::descriptor() on trait + 5 impls
- PolicyArtifact::from_policy(), save(), load(), into_policy(), validate()
- Round-trip tests for every policy type

**Phase 2 — Algorithm integration**
- Algorithm::policy_artifact() on trait + 5 impls
- RunResult gains artifact field
- CompetitionResult gains save_artifacts(), best_for_task()
- Competition tests verify artifacts

**Phase 3 — Critic descriptors + checkpoint types**
- NetworkDescriptor, NetworkKind
- ValueFn::descriptor() + 3 impls
- QFunction::descriptor() + 3 impls
- TrainingCheckpoint, NetworkSnapshot, OptimizerSnapshot types
- Algorithm::checkpoint() on trait + 5 todo!() impls
- Optimizer::snapshot() + load_snapshot()

**Phase 4 — Visual proof**
- Train-then-replay Bevy example (CEM on reaching-2dof)
- Trains inline (~20 epochs), switches to Bevy visualization
- Saves artifact to disk as side effect

**Phase 5 — Checkpoint implementations (per-algorithm, as needed)**
- CEM checkpoint (simple: policy + noise_std)
- REINFORCE checkpoint (policy + optimizer + sigma)
- PPO checkpoint (policy + value + optimizers)
- TD3 checkpoint (policy + targets + Q1/Q2 + targets + optimizers)
- SAC checkpoint (full state)
