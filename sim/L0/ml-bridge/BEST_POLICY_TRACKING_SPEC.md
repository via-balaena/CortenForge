# Best-Policy Tracking — Algorithm-Internal Best-Epoch Snapshot

> Every algorithm knows when it just had its best epoch. Snapshotting the
> weights at that moment is trivial. This makes "give me your best" a
> first-class capability — not a runner-level hack, not a post-hoc scan.
> The algorithm owns its own best performance.

**Status**: v7 — All phases complete (commits `c87fdbc`–`877a9cb`).
**Crate**: `sim-ml-bridge`
**Depends on**: Policy Persistence (complete — commit `2919732`)
**New dependencies**: None

---

## 1. Design principles

These extend the Policy Persistence spec's principles — same philosophy,
narrower scope.

| Principle | What it means |
|-----------|---------------|
| **Algorithm-owned** | The algorithm tracks its own best. Not the runner, not the caller, not a post-hoc scan of the metrics vec. The policy knows its own weights; the training loop knows when reward improved. Put the knowledge where the data is. |
| **Composed, not duplicated** | One shared `BestTracker` type encapsulates the snapshot logic — comparison, cloning, artifact construction. Each algorithm embeds it as a single field. The tracking decision, tie-breaking rule, and NaN handling live in one place. |
| **Structural** | `best_artifact()` is on the `Algorithm` trait — you can't implement Algorithm without implementing it. Same as `policy_artifact()` and `checkpoint()`. |
| **Zero-cost for CEM** | CEM is monotonic (elite selection only keeps improvements). Its best-epoch snapshot always equals its final policy. The tracking still happens — uniformity matters more than skipping a no-op clone. |
| **Non-breaking** | `policy_artifact()` semantics are unchanged (returns current/final policy). `best_artifact()` is additive. No existing caller breaks. |
| **Checkpoint-aware** | Best-policy state survives `checkpoint()` → `from_checkpoint()`. Resume training from epoch 30, and the best from epoch 17 is still remembered. |

---

## 2. Problem statement

After `Algorithm::train()` returns, `policy_artifact()` returns the
**final epoch's** weights. For algorithms that converge monotonically
(CEM), final = best. For gradient methods (TD3, SAC, PPO, REINFORCE),
the final epoch may be worse than an earlier peak.

**What's missing:**

| Gap | Impact |
|-----|--------|
| No best-epoch weights | Can't deploy the policy that actually performed best |
| No best-epoch metadata | Can't tell which epoch was best without scanning the metrics vec manually |
| Comparison uses final reward | Competition rankings use `final_reward()`, which penalizes algorithms that peak early then oscillate |
| Checkpoint loses best | Resume training → best-epoch tracking resets, previous best forgotten |

**What already exists (and why it's not enough):**

The `TrainingProvenance.metrics` vec stores every epoch's `mean_reward`.
A caller *can* scan it to find the best epoch's reward. But:
1. The caller doesn't have the **weights** from that epoch — they're gone.
2. Scanning metrics is a consumer concern — it shouldn't be repeated by
   every caller (competition runner, Bevy examples, analysis tools).
3. The algorithm had the weights at the moment of the best epoch. Not
   saving them is waste.

---

## 3. Core type: `BestTracker`

The entire tracking mechanism lives in a single composable type. Every
algorithm embeds it as one field instead of managing three loose fields
and duplicating the comparison/snapshot/artifact logic.

### 3.1 Type definition

```rust
/// Tracks the best-performing policy weights seen during training.
///
/// Embedded as a single field in every algorithm struct. Encapsulates
/// the snapshot decision (strict `>`), the params clone, and artifact
/// construction. One type, one set of rules, used by all 5 algorithms.
///
/// # Initialization
///
/// Created from the initial policy params in `Algorithm::new()`. Before
/// training, `reward` is `NEG_INFINITY` — the first real epoch always
/// overwrites it.
///
/// # Checkpoint round-trip
///
/// `to_checkpoint()` → `from_checkpoint()` preserves the best state
/// across training sessions.
#[derive(Debug, Clone)]
pub struct BestTracker {
    /// Policy weights at the best epoch.
    params: Vec<f64>,
    /// Mean reward at the best epoch.
    reward: f64,
    /// Epoch index (0-based) that achieved the best reward.
    epoch: usize,
}
```

**Derives**: `Debug` for diagnostic printing, `Clone` for checkpoint
convenience. No `Serialize`/`Deserialize` — `BestTracker` is never
serialized directly; its data flows through `to_checkpoint()` into
checkpoint fields and through provenance fields. `Send + Sync` is
automatic from field types (`Vec<f64>`, `f64`, `usize`), satisfying
the `Algorithm: Send` trait bound.

**Visibility**: The module is declared `pub(crate) mod best_tracker` in
`lib.rs`. Items inside use `pub` (not `pub(crate)`) because clippy
rejects redundant `pub(crate)` inside a restricted module. The module-
level visibility controls access — external crates cannot reach
`BestTracker`. The public surface is the `best_artifact()` trait method
and the provenance fields.

### 3.2 Constructor

```rust
impl BestTracker {
    /// Create a new tracker seeded with initial policy params.
    ///
    /// `NEG_INFINITY` reward guarantees the first real epoch always wins.
    pub fn new(initial_params: &[f64]) -> Self {
        Self {
            params: initial_params.to_vec(),
            reward: f64::NEG_INFINITY,
            epoch: 0,
        }
    }
}
```

Each algorithm calls this in `new()`:

```rust
// In Cem::new(), Reinforce::new(), Ppo::new(), Td3::new(), Sac::new():
best: BestTracker::new(policy.params()),
```

### 3.3 Snapshot method

```rust
impl BestTracker {
    /// Update the snapshot if this epoch's reward is strictly better.
    ///
    /// Strict `>` means ties keep the earlier snapshot — the policy that
    /// achieved a reward first (with less training) is more conservative.
    ///
    /// NaN rewards: `NaN > anything` is always `false` in IEEE 754, so
    /// NaN epochs are silently skipped. This is correct — NaN rewards
    /// indicate a training bug, not a best-epoch candidate.
    pub fn maybe_update(&mut self, epoch: usize, reward: f64, params: &[f64]) {
        if reward > self.reward {
            self.reward = reward;
            self.epoch = epoch;
            self.params = params.to_vec();
        }
    }
}
```

Each algorithm calls this once per epoch in `train()`:

```rust
// One line in the training loop, after mean_reward is computed:
self.best.maybe_update(epoch, mean_reward, self.policy.params());
```

### 3.4 Artifact construction

```rust
impl BestTracker {
    /// Build a bare `PolicyArtifact` from the best-epoch weights.
    ///
    /// The descriptor comes from the caller (the current policy's
    /// architecture) — architecture doesn't change during training,
    /// only weights change. Provenance is `None` — the caller attaches it.
    pub fn to_artifact(&self, descriptor: PolicyDescriptor) -> PolicyArtifact {
        PolicyArtifact {
            version: CURRENT_VERSION,
            descriptor,
            params: self.params.clone(),
            provenance: None,
        }
    }
}
```

Each algorithm's `best_artifact()` delegates to this:

```rust
fn best_artifact(&self) -> PolicyArtifact {
    self.best.to_artifact(self.policy.descriptor())
}
```

### 3.5 Accessors

```rust
impl BestTracker {
    /// Best-epoch mean reward (`NEG_INFINITY` if no training has occurred).
    pub const fn reward(&self) -> f64 {
        self.reward
    }

    /// Best-epoch index (0-based).
    pub const fn epoch(&self) -> usize {
        self.epoch
    }
}
```

Used by `checkpoint()` and by the competition runner when building
provenance.

### 3.6 Checkpoint support

```rust
impl BestTracker {
    /// Serialize the best state into checkpoint-compatible fields.
    ///
    /// Returns `Option<f64>` for reward: finite → `Some(reward)`,
    /// non-finite (NEG_INFINITY before training) → `None`.
    /// `serde_json` rejects non-finite f64 values, so `NEG_INFINITY`
    /// must never reach the serializer.
    pub fn to_checkpoint(&self) -> (Vec<f64>, Option<f64>, usize) {
        let reward = if self.reward.is_finite() { Some(self.reward) } else { None };
        (self.params.clone(), reward, self.epoch)
    }

    /// Restore from checkpoint fields.
    ///
    /// - If `params` is `None` (pre-feature checkpoint), falls back to
    ///   `fallback_params` (the checkpoint's current policy params).
    /// - If `params` length doesn't match `fallback_params` length
    ///   (architecture mismatch from incompatible checkpoint), discards
    ///   `params` and falls back to `fallback_params`.
    /// - If `reward` is `None` (pre-feature checkpoint or pre-training
    ///   checkpoint), restores to `NEG_INFINITY` — the first real epoch
    ///   will overwrite it.
    pub fn from_checkpoint(
        params: Option<Vec<f64>>,
        reward: Option<f64>,
        epoch: usize,
        fallback_params: &[f64],
    ) -> Self {
        let params = params
            .filter(|p| p.len() == fallback_params.len())
            .unwrap_or_else(|| fallback_params.to_vec());
        Self {
            params,
            reward: reward.unwrap_or(f64::NEG_INFINITY),
            epoch,
        }
    }
}
```

### 3.7 Module placement

`BestTracker` lives in a new file: `src/best_tracker.rs`. It's a
focused, single-responsibility module — no other types, no complex
dependencies.

Imports needed: `PolicyDescriptor`, `PolicyArtifact`, `CURRENT_VERSION`
from `crate::artifact`.

Module declared as `pub(crate) mod best_tracker` in `lib.rs` — not
re-exported publicly. Items inside the module use `pub` (not
`pub(crate)`) to satisfy clippy; module-level visibility controls
external access.

---

## 4. Algorithm trait — add `best_artifact()`

```rust
pub trait Algorithm: Send {
    fn name(&self) -> &'static str;

    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics>;

    fn policy_artifact(&self) -> PolicyArtifact;    // UNCHANGED — returns final/current

    fn checkpoint(&self) -> TrainingCheckpoint;     // UNCHANGED signature

    /// Extract the best-epoch policy as a portable artifact.
    ///
    /// Returns the policy weights from the epoch with the highest
    /// `mean_reward` seen during training. Before `train()` is called,
    /// returns the initial policy (same as `policy_artifact()`).
    ///
    /// Returns a **bare** artifact: descriptor + best params,
    /// provenance = None. The caller attaches provenance.
    fn best_artifact(&self) -> PolicyArtifact;
}
```

**Object safety**: `best_artifact(&self) -> PolicyArtifact` — no `Self`
in return position, no generics. Works with `Box<dyn Algorithm>`.

**Per-algorithm implementation** (identical one-liner for all 5):

```rust
fn best_artifact(&self) -> PolicyArtifact {
    self.best.to_artifact(self.policy.descriptor())
}
```

---

## 5. Per-algorithm integration

Each algorithm gains one field and three touch points: constructor,
training loop, trait method. The `BestTracker` type absorbs all logic.

### 5.1 CEM

```rust
pub struct Cem {
    policy: Box<dyn Policy>,
    hyperparams: CemHyperparams,
    noise_std: f64,
    best: BestTracker,                          // NEW
}

impl Cem {
    pub fn new(policy: Box<dyn Policy>, hyperparams: CemHyperparams) -> Self {
        let noise_std = hyperparams.noise_std;
        Self {
            best: BestTracker::new(policy.params()),  // before policy moves
            policy,
            hyperparams,
            noise_std,
        }
    }
}
```

**Note on field ordering in `new()`**: `BestTracker::new(policy.params())`
must be computed before `policy` moves into the struct. Rust initializes
fields in the order written in the struct literal, and `policy.params()`
borrows `policy`. The `best` field must appear before `policy` in the
struct literal (or be bound to a local first). All 5 algorithms face
this — the pattern is:

```rust
// Option A: field ordering (best before policy in struct literal)
Self {
    best: BestTracker::new(policy.params()),
    policy,
    // ...
}

// Option B: local binding
let best = BestTracker::new(policy.params());
Self {
    policy,
    best,
    // ...
}
```

Option B is clearer — it avoids relying on struct literal evaluation
order. All 5 algorithms should use Option B.

**Train loop insertion** (`cem.rs`, after `self.policy.set_params(&mean_params)`):

```rust
self.policy.set_params(&mean_params);

// Snapshot best-epoch policy.
self.best.maybe_update(epoch, mean_reward, self.policy.params());

self.noise_std = (self.noise_std * hp.noise_decay).max(hp.noise_min);
```

**CEM ordering note**: The snapshot captures params AFTER
`set_params(&mean_params)`, so it's the elite mean — the "real" policy.
The `mean_reward` that triggers it is the population-wide average, which
is the standard metric reported in `EpochMetrics.mean_reward`.

### 5.2 REINFORCE

```rust
pub struct Reinforce {
    policy: Box<dyn DifferentiablePolicy>,
    optimizer_config: OptimizerConfig,
    hyperparams: ReinforceHyperparams,
    optimizer: Box<dyn crate::optimizer::Optimizer>,
    sigma: f64,
    best: BestTracker,                          // NEW
}
```

**Train loop insertion**: After `mean_reward` computation, before
`on_epoch()`.

### 5.3 PPO

```rust
pub struct Ppo {
    policy: Box<dyn DifferentiablePolicy>,
    value_fn: Box<dyn ValueFn>,
    optimizer_config: OptimizerConfig,
    hyperparams: PpoHyperparams,
    actor_opt: Box<dyn crate::optimizer::Optimizer>,
    critic_opt: Box<dyn crate::optimizer::Optimizer>,
    sigma: f64,
    best: BestTracker,                          // NEW
}
```

### 5.4 TD3

```rust
pub struct Td3 {
    policy: Box<dyn DifferentiablePolicy>,
    target_policy: Box<dyn DifferentiablePolicy>,
    q1: Box<dyn QFunction>,
    q2: Box<dyn QFunction>,
    target_q1: Box<dyn QFunction>,
    target_q2: Box<dyn QFunction>,
    optimizer_config: OptimizerConfig,
    hyperparams: Td3Hyperparams,
    actor_opt: Box<dyn crate::optimizer::Optimizer>,
    q1_opt: Box<dyn crate::optimizer::Optimizer>,
    q2_opt: Box<dyn crate::optimizer::Optimizer>,
    best: BestTracker,                          // NEW
}
```

### 5.5 SAC

```rust
pub struct Sac {
    policy: Box<dyn StochasticPolicy>,
    q1: Box<dyn QFunction>,
    q2: Box<dyn QFunction>,
    target_q1: Box<dyn QFunction>,
    target_q2: Box<dyn QFunction>,
    optimizer_config: OptimizerConfig,
    hyperparams: SacHyperparams,
    actor_opt: Box<dyn crate::optimizer::Optimizer>,
    q1_opt: Box<dyn crate::optimizer::Optimizer>,
    q2_opt: Box<dyn crate::optimizer::Optimizer>,
    log_alpha: f64,
    best: BestTracker,                          // NEW
}
```

### 5.6 `from_checkpoint()` updates

All 5 algorithms restore the `BestTracker` from checkpoint fields:

```rust
// In Cem::from_checkpoint(), Reinforce::from_checkpoint(), etc.:
let best = BestTracker::from_checkpoint(
    checkpoint.best_params.clone(),
    checkpoint.best_reward,      // Option<f64> — None restores to NEG_INFINITY
    checkpoint.best_epoch,
    &checkpoint.policy_artifact.params,   // fallback for pre-feature or mismatched checkpoints
);
```

### 5.7 `checkpoint()` updates

All 5 algorithms serialize the `BestTracker` state:

```rust
fn checkpoint(&self) -> TrainingCheckpoint {
    let (best_params, best_reward, best_epoch) = self.best.to_checkpoint();
    TrainingCheckpoint {
        // ... existing fields unchanged ...
        best_params: Some(best_params),
        best_reward,    // Option<f64> — None if never trained
        best_epoch,
    }
}
```

---

## 6. TrainingProvenance — add `best_reward` and `best_epoch`

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingProvenance {
    // ... all existing fields unchanged ...

    /// Best epoch's mean reward (highest across all epochs).
    /// `None` = no data (0-epoch training, or pre-feature artifact).
    /// `Some(x)` = real best reward from training.
    #[serde(default)]
    pub best_reward: Option<f64>,
    /// Epoch index that achieved the best reward (0-based).
    #[serde(default)]
    pub best_epoch: usize,
}
```

Both fields use `#[serde(default)]` — existing serialized artifacts
without these fields will deserialize with `best_reward = None` and
`best_epoch = 0`. `None` unambiguously indicates "no data"
(distinguishable from a real reward of 0.0). No version bump needed
(forward-compatible per §6.3 of the persistence spec).

**Why `Option<f64>`, not `f64`**: All current tasks produce negative
rewards. If `best_reward` defaulted to `0.0`, it would look better
than any real training result — readers comparing rewards would see a
phantom "best" that never existed. `None` is unambiguous: the caller
knows the field has no data vs. the policy genuinely achieved 0.0.

**Why on provenance, not on the artifact**: Provenance is "who trained
it, how, and when." Best-epoch metadata is training history — it belongs
with the training record, not with the weights. The weights themselves
are in `best_artifact.params`.

**Finite-value policy**: If `best_reward` is `Some(x)`, `x` must be
finite in serialized provenance. The `validate()` method (persistence
spec §6.6) needs an additional check:

```rust
// In PolicyArtifact::validate(), after existing provenance checks:
if let Some(br) = prov.best_reward {
    if !br.is_finite() {
        return Err(ArtifactError::NonFiniteValue {
            field: "provenance.best_reward".into(),
        });
    }
}
```

**0-epoch case**: If no epochs ran, `best_reward` in provenance is
`None`. Callers can distinguish "no data" from "reward was literally
0.0."

---

## 7. Competition integration

### 7.1 `RunResult` gains `best_artifact`

```rust
pub struct RunResult {
    pub task_name: String,
    pub algorithm_name: String,
    pub metrics: Vec<EpochMetrics>,
    pub artifact: PolicyArtifact,           // final epoch (UNCHANGED)
    pub best_artifact: PolicyArtifact,      // NEW — best epoch
}
```

### 7.2 `RunResult` gains `best_reward()` helper

```rust
impl RunResult {
    /// Best epoch's mean reward, or `None` if no epochs ran.
    #[must_use]
    pub fn best_reward(&self) -> Option<f64> {
        self.metrics
            .iter()
            .map(|m| m.mean_reward)
            .max_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
    }
}
```

Computed from `metrics` (not from the stored `best_artifact`), so it's
independently verifiable.

### 7.3 Competition runner changes in `run()`

```rust
// After train() returns, extract both artifacts:
let artifact = algorithm.policy_artifact();
let best_artifact = algorithm.best_artifact();

// Build provenance (shared across both):
// Strict `>` matches BestTracker (§3.3): ties keep the earlier epoch.
let (best_epoch, best_reward) = {
    let mut bi = 0;
    let mut br = f64::NEG_INFINITY;
    for (i, m) in metrics.iter().enumerate() {
        if m.mean_reward > br {
            br = m.mean_reward;
            bi = i;
        }
    }
    if metrics.is_empty() {
        (0, None)
    } else if br.is_finite() {
        (bi, Some(br))
    } else {
        (0, None) // all NaN (shouldn't happen — assert_finite guards)
    }
};

let provenance = TrainingProvenance {
    algorithm: name.to_string(),
    task: task.name().to_string(),
    seed: self.seed,
    epochs_trained: metrics.len(),
    final_reward: metrics.last().map_or(0.0, |m| m.mean_reward),
    best_reward,
    best_epoch,
    total_steps: metrics.iter().map(|m| m.total_steps).sum(),
    wall_time_ms: metrics.iter().map(|m| m.wall_time_ms).sum(),
    timestamp: now_iso8601(),
    hyperparams: BTreeMap::new(),
    metrics: metrics.clone(),
    parent: None,
};

let artifact = artifact.with_provenance(provenance.clone());
let best_artifact = best_artifact.with_provenance(provenance);

runs.push(RunResult {
    task_name: task.name().to_string(),
    algorithm_name: name.to_string(),
    metrics,
    artifact,
    best_artifact,
});
```

### 7.4 `CompetitionResult` changes

- `save_artifacts()` saves both: `{task}_{algorithm}.artifact.json`
  (final) and `{task}_{algorithm}.best.artifact.json` (best).
- `best_for_task()` uses `best_reward()` instead of `final_reward()`.
- `print_ranked()` shows both final and best reward columns.

**Updated `print_ranked` format:**

```
=== Phase 6c: Obstacle avoidance (50ep/50env, 2-layer autograd) ===
Algorithm      Final Reward   Best Reward  Best @    Dones
------------------------------------------------------------
CEM                  -0.41         -0.41     49        12
TD3                  -0.55         -0.48     38         8
SAC                  -0.97         -0.82     41         5
PPO                -269.00       -245.00     12         0
REINFORCE          -269.00       -260.00      8         0

Ordering by best (best → worst): CEM > TD3 > SAC > PPO > REINFORCE
```

---

## 8. `TrainingCheckpoint` — add best-policy fields

```rust
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrainingCheckpoint {
    // ... all existing fields unchanged ...

    /// Best-epoch policy weights (for best_artifact() after resume).
    #[serde(default)]
    pub best_params: Option<Vec<f64>>,
    /// Best-epoch mean reward. `None` = no data (pre-feature or pre-training).
    #[serde(default)]
    pub best_reward: Option<f64>,
    /// Best-epoch index (0-based, relative to start of training).
    #[serde(default)]
    pub best_epoch: usize,
}
```

All three fields use `#[serde(default)]`:
- `best_params`: `None` → `BestTracker::from_checkpoint()` falls back to
  the checkpoint's policy params.
- `best_reward`: `None` → `BestTracker` restores to `NEG_INFINITY`
  internally (first epoch will overwrite). Avoids the `0.0` ambiguity
  problem (§6).
- `best_epoch`: `0` → conservative default.

`best_params` is additionally validated by `from_checkpoint()`: if the
length doesn't match the policy's param count (architecture mismatch),
it's discarded and falls back to current policy params (§3.6).

---

## 9. Memory cost analysis

The only new allocation is one `Vec<f64>` per algorithm — the best-epoch
params snapshot inside `BestTracker`.

| Policy type | Params | Snapshot size | Notes |
|-------------|--------|---------------|-------|
| LinearPolicy (2-DOF) | 6 | 48 bytes | Negligible |
| LinearPolicy (6-DOF) | 78 | 624 bytes | Negligible |
| MlpPolicy (6-DOF, H=32) | 2,118 | ~17 KB | Negligible |
| AutogradPolicy (6-DOF, [64,64]) | 5,958 | ~48 KB | Negligible |
| AutogradPolicy (21-DOF, [256,256]) | ~76K | ~608 KB | Still fine |

The clone only happens when reward improves — monotonic algorithms
(CEM) clone every epoch; oscillating algorithms clone less frequently
as they converge. Worst case is one clone per epoch, which is already
cheaper than the `on_epoch` callback's closure invocation.

---

## 10. CEM monotonicity analysis

CEM's elite selection guarantees that `mean_params` (and therefore the
policy) can only improve or stay the same. The population mean reward
can fluctuate (noisy random candidates), but the elite mean tracks
upward.

However, `mean_reward` as reported by CEM is the **population mean**
(all candidates, not just elites), computed at `cem.rs:198`:

```rust
let mean_reward: f64 = fitness.iter().map(|(_, f)| f).sum::<f64>() / n_envs as f64;
```

This CAN decrease epoch-to-epoch because bad candidates pull the
population average down even as the elite improves. So even for CEM,
`best_reward` tracking is meaningful — it captures the epoch where the
population as a whole performed best, not just the elites.

The `best_params` will still be the elite mean from that epoch (because
`set_params` happens before the snapshot), which is the correct policy
to save.

---

## 11. Interaction with existing features

### 11.1 `policy_artifact()` — unchanged

Returns the current/final policy. All existing callers continue to work.
No semantic change.

### 11.2 `checkpoint()` — additive

Three new fields with `#[serde(default)]`. Existing checkpoints
deserialize correctly (fields default to `None`/`0.0`/`0`).

### 11.3 `from_checkpoint()` — backward compatible

Missing `best_params` falls back to the checkpoint's current policy
params via `BestTracker::from_checkpoint()`. No error, no warning —
just a conservative default.

### 11.4 Curriculum / transfer learning

When loading a parent artifact to warm-start a new training run, the
new algorithm's `BestTracker` is initialized from the loaded policy's
params (via `new()`). The parent's best-epoch history lives in the
provenance chain — it's preserved but doesn't affect the new training's
best-tracking.

### 11.5 `train()` called twice

If `train()` is called twice on the same algorithm (continuous training),
`best.reward` persists from the first run. The second run only
overwrites the best if it finds a better epoch. This is correct — the
best across both runs is tracked.

The `best.epoch` counter is epoch-local (0-based within each `train()`
call). After two `train()` calls of 50 epochs each, `best.epoch` might
be 30 (from the first run) or 15 (from the second). It's always
relative to the `train()` call that produced it. This matches how
`EpochMetrics.epoch` works (0-based per call).

**Edge case**: If a second `train()` call resets epoch to 0 and beats
the first run's best at epoch 5, `best.epoch` becomes 5. The caller
can't distinguish "epoch 5 of run 2" from "epoch 5 of run 1." This is
acceptable because:
1. Calling `train()` twice is rare (checkpointing is the standard
   resumption mechanism).
2. The provenance `metrics` vec contains the full history from the most
   recent `train()` call — the caller can correlate `best_epoch` to the
   metrics.

---

## 12. Implementation scope

### New type
- `BestTracker` in `src/best_tracker.rs` (~120 LOC incl. tests)

### Trait change (breaking)
- `Algorithm::best_artifact()` — 5 concrete impls + test mocks

### Per-algorithm changes (1 field + 4 touch points each)
- 5 structs: add `best: BestTracker` field
- 5 `new()` constructors: add `BestTracker::new(policy.params())`
- 5 `from_checkpoint()` constructors: add `BestTracker::new(policy.params())`
  in Phase 1, upgraded to `BestTracker::from_checkpoint(...)` in Phase 3
- 5 `train()` loops: add `self.best.maybe_update(epoch, mean_reward, self.policy.params())`
- 5 `best_artifact()` impls: `self.best.to_artifact(self.policy.descriptor())`
- 5 `checkpoint()` impls: add `self.best.to_checkpoint()` fields (Phase 3)

### Type changes (additive, forward-compatible)
- `TrainingProvenance` gains `best_reward: Option<f64>`,
  `best_epoch: usize` (both `#[serde(default)]`)
- `TrainingCheckpoint` gains `best_params: Option<Vec<f64>>`,
  `best_reward: Option<f64>`, `best_epoch: usize`
  (all `#[serde(default)]`)

### Competition changes
- `RunResult` gains `best_artifact: PolicyArtifact`
- `RunResult` gains `best_reward()` method
- `Competition::run()` extracts best artifact + builds provenance
- `CompetitionResult::save_artifacts()` saves best artifacts too
- `CompetitionResult::best_for_task()` uses best reward
- `CompetitionResult::print_ranked()` shows both columns

### Validation
- `PolicyArtifact::validate()` gains `best_reward` finite check

### Test mock updates
- `MockAlgorithm` in `competition.rs` tests: add `best: BestTracker`
  field + `best_artifact()` impl
- Any `RunResult` construction in tests: add `best_artifact` field

### Example updates
- `examples/fundamentals/sim-ml/persistence/train-then-replay/src/main.rs`
  — `TrainingProvenance` struct literal at line 157 needs `best_reward`
  and `best_epoch` fields. This example constructs provenance directly
  (does not impl `Algorithm`).

### Implementation note: `PolicyDescriptor.activation`
`PolicyDescriptor` has an `activation: Activation` field (from
`autograd_layers`) not shown in spec code snippets. Any test helper
constructing a `PolicyDescriptor` struct literal must include it
(e.g. `activation: Activation::Tanh`).

### Implementation note: struct literal cascading
Adding fields to `TrainingProvenance` forces updates to every struct
literal in the codebase — not just the Phase 2 files. In practice
this pulled all Phase 4 work (example + `artifact.rs` test literals)
into Phase 2. Rust won't compile with missing fields, so this is
unavoidable. Future specs should not phase struct-literal updates
separately from the struct change.

### Implementation note: `print_ranked` best_epoch
`print_ranked()` computes best epoch inline from `metrics` via
`enumerate().max_by()` rather than reading `provenance.best_epoch`.
This is intentional — `print_ranked` operates on `RunResult` (which
has `metrics` directly) and avoids unwrapping provenance. The inline
computation is consistent with `best_reward()` also computing from
metrics.

### Tests

**Phase 1** (10 tests — done):
- `new_starts_at_neg_infinity` — constructor state
- `maybe_update_improves` — improves, ties (strict `>`), worse skipped
- `maybe_update_skips_nan` — `NaN` epoch silently skipped
- `to_artifact_builds_correctly` — version, params, provenance=None
- `checkpoint_round_trip` — to_checkpoint → from_checkpoint preserves state
- `checkpoint_pre_training_maps_neg_infinity_to_none` — serde safety
- `from_checkpoint_with_none_params_falls_back` — pre-feature compat
- `from_checkpoint_with_mismatched_length_falls_back` — arch mismatch
- `best_artifact_before_train_returns_initial_policy` — integration
- `best_artifact_after_training_has_best_epoch_weights` — integration

**Phase 2** (4 tests — done):
- `run_result_best_reward_matches_manual_scan` — best vs final divergence
- `provenance_best_fields_serde_round_trip` — JSON round-trip preserves both fields
- `save_artifacts_writes_best_files` — `.best.artifact.json` written + loadable
- `validate_rejects_non_finite_best_reward` — `Infinity` in `best_reward` rejected

**Phase 3** (3 tests — done):
- `checkpoint_round_trip_preserves_best_across_resume` — train → checkpoint → restore → verify best artifact matches
- `old_checkpoint_without_best_fields_loads_with_defaults` — stripped fields → JSON round-trip → fallback to policy params
- `resume_training_preserves_best_from_previous_session` — session 1 best survives resume if session 2 doesn't beat it

**Total**: 17 tests (all done)

---

## 13. Phasing

**Phase 1 — `BestTracker` + trait surgery** ✅ (commit `c87fdbc`, `66dcc46`)
- Create `src/best_tracker.rs` with full `BestTracker` impl
- Add `best: BestTracker` field to all 5 algorithm structs
- Initialize in `new()` from initial policy params
- Initialize in `from_checkpoint()` with `BestTracker::new(policy.params())`
  (placeholder — Phase 3 upgrades to `BestTracker::from_checkpoint()`)
- Add `self.best.maybe_update(...)` to all 5 `train()` loops
- Add `best_artifact()` to `Algorithm` trait + 5 impls
- Update `MockAlgorithm`
- `#[allow(dead_code)]` on `reward()`, `epoch()`, `to_checkpoint()`,
  `from_checkpoint()` — used in Phase 2/3 but not yet called
- 10 tests (8 BestTracker unit + 2 integration: pre-train + post-train)

Verification: `cargo test -p sim-ml-bridge --lib` — 375 passed

**Phase 2 — Provenance + competition integration** ✅ (commit `b7a4e20`)
- Add `best_reward`, `best_epoch` to `TrainingProvenance`
- Add `best_artifact` to `RunResult`, `best_reward()` helper
- Update `Competition::run()` to extract best artifact
- Update `print_ranked()` to show both columns (Final Reward, Best
  Reward, Best @, Dones — sorted by best reward)
- Update `save_artifacts()` and `best_for_task()` (returns
  `&r.best_artifact` now, not `&r.artifact`)
- Add `best_reward` validation to `PolicyArtifact::validate()`
- Update test `RunResult` constructions (5 sites)
- 4 unit tests
- **Pulled forward from Phase 4**: updated `train-then-replay` example
  provenance literal + 4 `TrainingProvenance` test literals in
  `artifact.rs` (required for compilation — Rust struct literals need
  all fields)
- `#[allow(dead_code)]` on `reward()`, `epoch()` remains — Phase 2
  competition runner computes best from metrics (independent
  verification), not from BestTracker accessors. `to_checkpoint()`,
  `from_checkpoint()` still dead_code (Phase 3).

Verification: `cargo test -p sim-ml-bridge --lib` — 379 passed.
`cargo test -p sim-ml-bridge --test competition` — 13 ignored (multi-
minute), 0 failed.

**Phase 3 — Checkpoint integration** ✅ (commit `877a9cb`)
- Add best state fields to `TrainingCheckpoint` (§8)
- Update all 5 `checkpoint()` impls to use `self.best.to_checkpoint()`
- Update all 5 `from_checkpoint()` impls to use `BestTracker::from_checkpoint()`
  (replace `BestTracker::new(policy.params())` placeholder from Phase 1)
- Update `MockAlgorithm::checkpoint()` in `competition.rs`
- Update `TrainingCheckpoint` struct literal in `artifact.rs` test
- Remove `#[allow(dead_code)]` from `BestTracker::to_checkpoint()`,
  `from_checkpoint()` — both now have non-test callers (5 algos + mock)
- `#[allow(dead_code)]` on `reward()`, `epoch()` **remains** —
  `to_checkpoint()` accesses the fields directly, not through the
  accessor methods. No non-test callers exist yet. Comment updated
  to "non-test callers pending (visualization / analysis)".
- 3 unit tests (round-trip, backward compat, resume tracking)

Verification: `cargo test -p sim-ml-bridge --lib` — 382 passed.

### Implementation note: `reward()`/`epoch()` accessors
The Phase 3 plan assumed `to_checkpoint()` would call `self.reward()`
and `self.epoch()`, giving them non-test callers. In practice,
`to_checkpoint()` accesses the private fields directly (it's in the
same struct impl block), so the accessors remain dead outside tests.
This is correct — adding method indirection just to remove an
`#[allow(dead_code)]` would be worse. These accessors will get real
callers when visualization or analysis code reads best-epoch state.

**Phase 4 — Example and external caller updates** ✅ (done in Phase 2)
- ~~Update `train-then-replay` example~~ — done in Phase 2 (compile-
  required: Rust struct literals need all fields)
- ~~Update `artifact.rs` test literals~~ — done in Phase 2 (same reason)
- No remaining work — Phase 4 is empty.

---

## 14. Codebase stress test

Issues to verify before implementation.

### 14.1 Verified assumptions

| Assumption | Evidence needed |
|------------|-----------------|
| `self.policy.params().to_vec()` is a valid snapshot | Verify `params()` returns `&[f64]` for all policy types behind trait objects |
| Descriptor doesn't change during training | Verify no algorithm modifies obs_dim, act_dim, hidden_dims, obs_scale during `train()` |
| `CURRENT_VERSION` is accessible from `best_tracker.rs` | Verify it's `pub` in `artifact.rs` |
| `mean_reward` variable exists in all 5 `train()` loops | Grep for `mean_reward` in each algorithm — verify it's the epoch-level metric |
| `epoch` variable is in scope at the snapshot point | Verify the loop variable name is `epoch` in all 5 algorithms |
| `PolicyArtifact` struct literal construction compiles | Verify all 4 fields (version, descriptor, params, provenance) are `pub` |
| `PolicyDescriptor` is importable in `best_tracker.rs` | Verify it's `pub` in `artifact.rs` |
| `TrainingProvenance` has no `deny_unknown_fields` | Grep for `deny_unknown_fields` in `artifact.rs` |
| `TrainingCheckpoint` has no `deny_unknown_fields` | Same grep |
| `RunResult` is only constructed in `competition.rs` | Grep for `RunResult {` workspace-wide |
| `mean_reward` is from env rollouts, not replay buffer (TD3/SAC) | TD3: `epoch_rewards` from completed episodes during env interaction (`td3.rs:328,476-480`). SAC: same pattern (`sac.rs:348,529-533`). Not from replay buffer or critic estimates. |

### 14.2 Potential issues

| Risk | Mitigation |
|------|------------|
| `policy.params()` borrow vs `policy` move in `new()` | Use local binding: `let best = BestTracker::new(policy.params());` before `Self { policy, best, .. }` |
| CEM's `policy.set_params()` before snapshot | Correct — we want the elite mean's params, not the last candidate's |
| TD3/SAC `mean_reward` — is it evaluation or training reward? | **Verified**: both compute from fresh environment rollouts (completed episodes during env interaction), not replay buffer. TD3: `td3.rs:476-480`, SAC: `sac.rs:529-533`. Safe to snapshot based on. |
| Stochastic policies: `params()` includes `log_std` | Correct — `log_std` is a learned parameter, should be snapshotted |
| `best.epoch` ambiguity after two `train()` calls | Documented in §11.5 as acceptable edge case |
| `BestTracker` needs `PolicyDescriptor` + `PolicyArtifact` imports | Both are `pub` in `artifact.rs`, importable via `crate::artifact` |
| `best_params` length mismatch on checkpoint load | `from_checkpoint()` validates length matches `fallback_params.len()` — discards on mismatch (§3.6) |

---

## 15. Spec grading rubric

Implementation begins only when every criterion reaches A+.

### Criterion 1: Codebase verification
> Every claim about existing code is verified against the actual source.

**Policy trait** (verify `params()` is accessible through each trait object):
- [x] `Policy::params(&self) -> &[f64]` — confirm exact signature *(policy.rs:35)*
- [x] `DifferentiablePolicy: Policy` — confirm supertrait, `params()` accessible *(policy.rs:80)*
- [x] `StochasticPolicy: DifferentiablePolicy` — confirm supertrait chain *(policy.rs:147)*

**Algorithm struct fields** (read each struct definition):
- [x] CEM: verify `policy: Box<dyn Policy>` *(cem.rs:58-63)*
- [x] REINFORCE: verify `policy: Box<dyn DifferentiablePolicy>` *(reinforce.rs:61-70)*
- [x] PPO: verify `policy: Box<dyn DifferentiablePolicy>` *(ppo.rs:70-82)*
- [x] TD3: verify `policy: Box<dyn DifferentiablePolicy>` *(td3.rs:82-98)*
- [x] SAC: verify `policy: Box<dyn StochasticPolicy>` *(sac.rs:87-104)*

**Training loop variables** (grep in each algorithm's `train()`):
- [x] CEM: `mean_reward` exists, `epoch` is loop var, `set_params` precedes reward *(cem.rs:142,194,198)*
- [x] REINFORCE: `mean_reward` exists, `epoch` is loop var *(reinforce.rs:164,263)*
- [x] PPO: `mean_reward` exists, `epoch` is loop var *(ppo.rs:221,417)*
- [x] TD3: `mean_reward` exists, `epoch` is loop var, reward is from env rollouts not replay buffer (`td3.rs:328,476-480`) *(epoch_rewards from completed episodes during env interaction)*
- [x] SAC: `mean_reward` exists, `epoch` is loop var, reward is from env rollouts not replay buffer (`sac.rs:348,529-533`) *(same pattern as TD3)*

**Existing types** (verify import paths):
- [x] `PolicyArtifact` — all 4 fields are `pub` *(artifact.rs:225-236)*
- [x] `PolicyDescriptor` — `pub` in `artifact.rs` *(artifact.rs:60-76)*
- [x] `CURRENT_VERSION` — `pub const` in `artifact.rs` *(artifact.rs:33)*
- [x] `TrainingProvenance` — no `deny_unknown_fields` *(artifact.rs:120-145, 11 fields, all pub)*
- [x] `TrainingCheckpoint` — no `deny_unknown_fields` *(artifact.rs:590-602, 5 fields, all pub)*
- [x] `RunResult` — only constructed in `competition.rs` (source + tests) *(line 343 production; lines 508,537,549,568,587 in tests)*

**Grade**: A+

### Criterion 2: Type system soundness
> Every proposed type, trait method, and impl is valid Rust.

- [x] `BestTracker` fields: `Vec<f64>`, `f64`, `usize` — all `Send + Sync`
- [x] `BestTracker` derives `Debug`, `Clone` — no `Serialize`/`Deserialize` (not serialized directly)
- [x] `BestTracker` is `pub(crate)` — accessible from `cem.rs`, `td3.rs`, etc. (same crate)
- [x] `best_artifact(&self) -> PolicyArtifact` — object-safe (no Self, no generics)
- [x] `policy.params()` borrow resolved before `policy` moves into struct (§5.1 Option B)
- [x] `TrainingProvenance` new fields: `Option<f64>` and `usize` — serde-friendly, `#[serde(default)]`
- [x] `TrainingCheckpoint` new fields: `Option<Vec<f64>>`, `Option<f64>`, `usize` — serde-friendly with `#[serde(default)]`
- [x] `PolicyArtifact` struct literal in `to_artifact()` — all 4 fields provided
- [x] `best_reward: f64` on `BestTracker` struct is fine (`NEG_INFINITY` never serialized); provenance uses `Option<f64>`, `validate()` checks `Some(x)` is finite
- [x] `to_checkpoint()` maps non-finite reward to `None` — `serde_json` never sees `NEG_INFINITY` (§3.6)

**Grade**: A+

### Criterion 3: Completeness
> No gaps — every path through the system is specified.

- [x] `best_artifact()` before `train()` — returns initial policy (§3.2 `NEG_INFINITY`)
- [x] `best_artifact()` after 0-epoch training — returns initial policy (no epochs → no update)
- [x] `best_artifact()` when all epochs have identical reward — returns epoch 0's weights (strict `>`)
- [x] `best_artifact()` after two `train()` calls — tracks best across both (§11.5)
- [x] `checkpoint()` + `from_checkpoint()` round-trip — best state preserved (§3.6, §5.6-5.7)
- [x] `from_checkpoint()` with pre-feature checkpoint — `BestTracker::from_checkpoint()` fallback (§3.6)
- [x] Provenance `best_reward` validation — must be finite (§6)
- [x] `save_artifacts()` file naming — both final and best saved (§7.4)
- [x] `BestTracker::maybe_update()` with NaN reward — silently skipped (§3.3)
- [x] `from_checkpoint()` with architecture-mismatched `best_params` — discards and falls back to policy params (§3.6)
- [x] `best_artifact()` after `from_checkpoint()` without retraining — returns restored best from previous session (correct: no training = no new best)

**Grade**: A+

### Criterion 4: Breaking change audit
> Every existing caller that breaks is identified with a migration path.

**Algorithm trait impls that need `best_artifact()`:**
- [x] `impl Algorithm for MockAlgorithm` (`competition.rs` tests) — needs `best: BestTracker` field + `best_artifact()` impl *(competition.rs:380-447)*
- [x] Any benchmarks that impl `Algorithm` — check `benches/` (none found)
- [x] Any examples that impl `Algorithm` — check `examples/` (none found — examples use algorithms, don't impl the trait)

**`RunResult` construction sites** (need `best_artifact` field):
- [x] `competition.rs` `run()` method — production construction *(line 343)*
- [x] `competition.rs` tests — all test `RunResult` struct literals *(lines 508, 537, 549, 568, 587)*

**`TrainingProvenance` construction sites** (need `best_reward: Option<f64>`, `best_epoch: usize`):
- [x] `competition.rs:329` — production (competition runner)
- [x] `examples/fundamentals/sim-ml/persistence/train-then-replay/src/main.rs:157` — production (Bevy example)
- [x] `artifact.rs:1055` — test (`validate_rejects_nan_final_reward`)
- [x] `artifact.rs:1082` — test (`with_provenance_attaches`)
- [x] `artifact.rs:1119` — test (`provenance_chain_round_trips`, grandparent)
- [x] `artifact.rs:1132` — test (`provenance_chain_round_trips`, parent)

**`TrainingCheckpoint` construction sites** (need `best_params`, `best_reward`, `best_epoch`):
- [x] `cem.rs:227` — `Cem::checkpoint()`
- [x] `reinforce.rs:290` — `Reinforce::checkpoint()`
- [x] `ppo.rs:453` — `Ppo::checkpoint()`
- [x] `td3.rs:510` — `Td3::checkpoint()`
- [x] `sac.rs:568` — `Sac::checkpoint()`
- [x] `competition.rs:439` — `MockAlgorithm::checkpoint()`
- [x] `artifact.rs:1161` — test (`checkpoint_save_load_round_trip`)

**Other:**
- [x] `validate()` in `artifact.rs` — needs `best_reward` finite check (`Some(x)` must be finite) *(validate() at artifact.rs:304-375 currently has no best_reward check — spec §6 adds it)*
- [x] Existing artifact file `trained_reaching_2dof.artifact.json` — loads correctly with new `#[serde(default)]` fields (confirmed: no `deny_unknown_fields`) *(file exists at repo root)*

**Grade**: A+

### Criterion 5: Internal consistency
> No contradictions between sections.

- [x] §3.2 `NEG_INFINITY` initialization matches §3.3 strict `>` snapshot condition
- [x] §3.4 `to_artifact()` matches §4 trait method usage
- [x] §5.6 `from_checkpoint()` uses §3.6 `BestTracker::from_checkpoint()`
- [x] §5.7 `checkpoint()` uses §3.6 `BestTracker::to_checkpoint()`
- [x] §6 provenance fields match §7.3 competition runner assembly *(v3 gap fixed: §7.3 `best_epoch` originally used `max_by` which returns last of equals on ties, contradicting BestTracker's strict `>`. Replaced with explicit `>` loop to match §3.3.)*
- [x] §8 checkpoint fields match §3.6 checkpoint methods (same 3 values)
- [x] §12 scope counts match §13 phasing counts (including Phase 4) *(Phase 1: BestTracker+trait+5 algos+mock = 7 tests; Phase 2: provenance+competition+validation = 4 tests; Phase 3: checkpoint = 3 tests; Phase 4: example+artifact.rs tests. Total ~14.)*
- [x] §9 memory analysis uses correct param counts (cross-check with persistence spec §7.1.1) *(AutogradPolicy [64,64] = 5,958 verified for obs_dim=21, act_dim=6. LinearPolicy 2-DOF = 6, 6-DOF = 78 verified. Orders of magnitude correct; conclusion "negligible" holds.)*
- [x] `best_reward` is `Option<f64>` everywhere it's serialized: §6 (provenance), §7.3 (runner), §8 (checkpoint), §3.6 (`to_checkpoint` return) — no bare `f64` for serialized best_reward

**Grade**: A+

### Criterion 6: Robustness
> Edge cases and real-world scenarios.

- [x] NaN `mean_reward`: `NaN > self.reward` is `false` (IEEE 754) — NaN epoch silently skipped
- [x] All epochs have negative reward: first epoch overwrites `NEG_INFINITY`, tracking correct
- [x] Single-epoch training: best = final = only epoch
- [x] `best.params` memory lifetime: `Vec` on `BestTracker`, dropped when algorithm drops
- [x] Thread safety: algorithms are `Send`, `BestTracker` fields are all `Send + Sync`
- [x] `best_reward` overflow: f64 can't overflow from normal reward values
- [x] `to_checkpoint()` with `NEG_INFINITY` `reward` (before training): **Handled** — `to_checkpoint()` maps non-finite reward to `None` (§3.6). `serde_json` never sees `NEG_INFINITY`.
- [x] `best_reward` default semantics: **Handled** — `Option<f64>` with `#[serde(default)]` gives `None` (not `0.0`). Callers can distinguish "no data" from "reward was literally 0.0" (§6).
- [x] `best_params` length mismatch on checkpoint load: **Handled** — `from_checkpoint()` validates `params.len() == fallback_params.len()`, discards on mismatch (§3.6).

**Grade**: A+

### Criterion 7: Future-proofing
> Extensibility verified by concrete analysis.

- [x] Adding a new algorithm (e.g., A2C): add `best: BestTracker` field + 1 `maybe_update` line + 1 `best_artifact` impl. Pattern documented in §5.
- [x] Adding evaluation-based best tracking: `BestTracker` tracks by training reward. A future `EvalBestTracker` would be a separate type — doesn't conflict.
- [x] Policy zoo: `best_artifact` enables saving the best policy per competition. The `.best.artifact.json` naming convention is extensible.
- [x] `BestTracker` could gain `maybe_update_eval()` for separate eval reward tracking — additive to the type, no changes to existing methods.

**Grade**: A+

### Overall

| Criterion | Grade | Notes |
|-----------|-------|-------|
| 1. Codebase verification | A+ | 16/16 — all claims verified with exact line numbers |
| 2. Type system soundness | A+ | 10/10 — ownership, object safety, serde boundaries all correct |
| 3. Completeness | A+ | 11/11 — every code path and edge case specified |
| 4. Breaking change audit | A+ | 14/14 — all 26 construction sites enumerated with line numbers |
| 5. Internal consistency | A+ | 9/9 — one gap found and fixed: §7.3 tie-breaking aligned with §3.3 strict `>` |
| 6. Robustness | A+ | 9/9 — NaN, NEG_INFINITY, serde, thread safety all handled |
| 7. Future-proofing | A+ | 4/4 — clear extension patterns, no conflicts |

**Implementation gate**: All 7 criteria must be A+ before Phase 1 begins.
