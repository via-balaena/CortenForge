# Physics-Aware ML Pivot — Construction Spec

This is the mechanical "what" to accompany `physics_aware_ml_pivot.md`'s
strategic "why." Every file, signature, and test assertion is spelled out
so the execution session makes **no design calls** on the fly — only
implementation choices inside `SimulatedAnnealing::train()` and the other
algorithm bodies.

Read this document top to bottom. Each numbered section maps to one commit.

---

## 0. Preconditions

- **Branch.** `feature/sim-ml-renovation`. Work continues off this branch;
  the split lands as one PR with the commit sequence below.
- **Starting state verified.** `sim/L0/ml-bridge/` has 28 `.rs` files in
  `src/`, plus `tests/competition.rs`, `tests/custom_task.rs`, and
  `benches/bridge_benchmarks.rs`. `sim-thermostat/Cargo.toml` has
  `sim-ml-bridge` as a **dev-dependency** (not a regular dep).
- **Never run.** `cargo xtask check` or `cargo test` at the workspace
  level — per CLAUDE.md, not during normal work. Per-crate
  `cargo xtask grade <name>` and targeted `cargo test -p <name>` only.
- **Release mode for heavy tests.** Any D2c rematch run uses `--release`
  (the existing D2c tests are gated behind `#[ignore = "requires --release"]`).

---

## 1. File manifest

### 1.1 New crates (created)

| Path | Purpose |
|---|---|
| `sim/L0/ml-chassis/Cargo.toml` | package = `sim-ml-chassis` |
| `sim/L0/ml-chassis/src/lib.rs` | chassis public surface |
| `sim/L0/ml-chassis/src/stats.rs` | **new** — `gaussian_log_prob` f64 helper, relocated from `sac.rs`. One pub function, ~15 lines. See §3.6. |
| `sim/L0/rl/Cargo.toml` | package = `sim-rl` (rename of `sim-ml-bridge`) |
| `sim/L0/rl/src/lib.rs` | algorithm crate root with re-exports |
| `sim/L0/rl/src/builders.rs` | `build_*` factories for `Competition` |
| `sim/L0/opt/Cargo.toml` | package = `sim-opt` |
| `sim/L0/opt/src/lib.rs` | opt crate root |
| `sim/L0/opt/src/annealing.rs` | `SimulatedAnnealing` + `AnnealingHyperparams` |
| `sim/L0/opt/src/tempering.rs` | stub — `todo!()` body, types only |
| `sim/L0/opt/src/cma_es.rs` | stub — `todo!()` body, types only |
| `sim/L0/opt/src/metropolis.rs` | stub — `todo!()` body, types only |
| `sim/L0/opt/src/builders.rs` | `build_simulated_annealing`, stubs for the rest |
| `sim/L0/opt/tests/d2c_rematch.rs` | integration test, §7 |

### 1.2 Moved files (`sim/L0/ml-bridge/src/*.rs` → `sim/L0/ml-chassis/src/`)

Moved verbatim, no content changes except path-rewrite imports and the
three targeted edits listed after the file block.

```
algorithm.rs           artifact.rs            autograd.rs
autograd_layers.rs     autograd_policy.rs     autograd_value.rs
best_tracker.rs        competition.rs         env.rs
error.rs               gae.rs                 linear.rs
lib.rs                 mlp.rs                 optimizer.rs
policy.rs              replay_buffer.rs       rollout.rs
space.rs               task.rs                tensor.rs
value.rs               vec_env.rs
```

Two targeted content edits during the move (commit 2):

1. **`best_tracker.rs` visibility bump** — `pub(crate) mod` → `pub mod` at lib.rs level. See §3.5.
2. **`autograd.rs:77` docstring** — rewrite `sim_ml_bridge::autograd::` → `sim_ml_chassis::autograd::`.

Note: `TaskConfig::from_build_fn` already exists upstream (added during the ml-chassis-refactor study, PR #190, with signature `fn from_build_fn<F>(name, obs_dim, act_dim, obs_scale, build_fn) -> Self where F: Fn(usize, u64) -> Result<VecEnv, EnvError> + Send + Sync + 'static`). No new signature work in this split — the function moves with `task.rs` unchanged.

Plus `sim/L0/ml-bridge/benches/bridge_benchmarks.rs` → `sim/L0/ml-chassis/benches/bridge_benchmarks.rs`. Grep-verified as chassis-only (no algorithm usage).

### 1.3 Files that stay in the renamed `sim-rl` crate

`sim/L0/ml-bridge/` becomes `sim/L0/rl/`. After the split, the following
remain in `sim/L0/rl/src/`:

```
cem.rs         reinforce.rs   ppo.rs         td3.rs         sac.rs
lib.rs         builders.rs (new)
```

And under `sim/L0/rl/tests/` — the two integration tests that exercise
algorithm structs:

```
custom_task.rs    (uses Cem::new directly — verified at
                   sim/L0/ml-bridge/tests/custom_task.rs:11,85)
competition.rs    (Phases 3+6 Cem/Ppo/Sac/Td3 comparison runs)
```

These **cannot** move to chassis: they import algorithm structs, and
putting them under `sim-ml-chassis/tests/` would require a
`sim-ml-chassis → sim-rl` dev-dep, creating a dev-dep cycle.
Content edit on the move: rewrite their `sim_ml_bridge::` imports to
`sim_rl::` (everything they need is already re-exported via sim-rl per
§4.2).

### 1.4 Content edits inside existing algorithm files (commit 2)

These edits happen in commit 2, **while the package is still named
`sim-ml-bridge` on disk**. Commit 3 is a pure rename + directory move;
by then the algorithm files already compile against `sim-ml-chassis`.
This is a sequencing requirement, not a stylistic choice: after commit
2 physically moves `algorithm.rs`, `best_tracker.rs`, `vec_env.rs`, etc.
out of `sim-ml-bridge/src/`, the remaining algorithm files' `use
crate::algorithm::Algorithm` lines stop resolving and the crate no
longer compiles. They have to be rewritten in the same commit.

- **`sim-ml-bridge/Cargo.toml`** — add `sim-ml-chassis = { workspace = true }`
  as a regular dep. (It will be dropped in commit 3 and replaced by
  the new `sim-rl` Cargo.toml per §4.1, so this is a one-commit-lifetime
  edit.)
- **All five algorithm files** (`cem.rs`, `ppo.rs`, `td3.rs`, `sac.rs`,
  `reinforce.rs`) — mechanical rewrite of `use crate::X` to
  `use sim_ml_chassis::X` for every chassis module (`algorithm`,
  `artifact`, `best_tracker`, `gae`, `optimizer`, `policy`,
  `replay_buffer`, `rollout`, `tensor`, `value`, `vec_env`). Do **not**
  rewrite intra-algorithm `crate::` paths (e.g., `crate::cem::Cem` stays).
  See §4.3 for the full list.
- **`sim-ml-bridge/src/sac.rs`** — delete the local `pub fn gaussian_log_prob`
  at `sac.rs:226-242` (moved to chassis per §3.6). Replace its two
  internal use-sites at `sac.rs:397,519` via
  `use sim_ml_chassis::stats::gaussian_log_prob;` at the top.
- **`sim-ml-bridge/src/lib.rs`** — drop the `pub mod` lines for the moved
  modules, drop their `pub use` re-exports, and drop
  `pub use sac::{..., gaussian_log_prob};` (since the function no longer
  lives here). Keep the algorithm module declarations and their
  `pub use` lines.
- **`ppo.rs:186`** has its own private `fn gaussian_log_prob` with a
  single-sigma signature (`fn gaussian_log_prob(mu: &[f64], action: &[f64], sigma: f64)`).
  It is **not** the same function — different signature, different
  use. Leave it untouched.

### 1.5 Files modified in place (outside the new crates)

| File | Change |
|---|---|
| `Cargo.toml` (root) | workspace members + `[workspace.dependencies]` aliases — see §9 |
| `sim/L0/thermostat/Cargo.toml` | replace `sim-ml-bridge` dev-dep with `sim-rl` dev-dep |
| `sim/L0/thermostat/tests/d1c_cem_training.rs` | import rewrite per §4 |
| `sim/L0/thermostat/tests/d1d_reinforce_comparison.rs` | import rewrite per §4 |
| `sim/L0/thermostat/tests/d2b_stochastic_resonance_baselines.rs` | import rewrite per §4 |
| `sim/L0/thermostat/tests/d2c_cem_training.rs` | import rewrite per §4 |
| 17 example crates under `examples/fundamentals/sim-ml/` | import rewrite per §4 |
| `docs/` grep hits | import rewrite per §4 |
| `docs/thermo_computing/01_vision/physics_aware_ml_pivot.md` | no further edits; already revised |

---

## 2. Workspace layout after the split

```
sim/L0/
├── types/              (unchanged)
├── simd/               (unchanged)
├── core/               (unchanged)
├── mjcf/               (unchanged)
├── urdf/               (unchanged)
├── gpu/                (unchanged)
├── tests/              (unchanged)
├── thermostat/         (modified — Cargo.toml dev-dep rename only)
├── ml-chassis/         (new, populated from ml-bridge split)
├── rl/                 (new, renamed from ml-bridge)
└── opt/                (new)
```

`sim/L0/ml-bridge/` **does not exist** after the PR merges.

---

## 3. `sim-ml-chassis` — public surface

### 3.1 Cargo.toml

```toml
[package]
name = "sim-ml-chassis"
description = "Algorithm chassis: traits, primitives, Competition runner — the foundation every CortenForge algorithm crate bolts onto"
version.workspace = true
edition.workspace = true
license.workspace = true
repository.workspace = true
authors.workspace = true
rust-version.workspace = true

[features]
bevy = ["dep:bevy_ecs"]

[dependencies]
bevy_ecs    = { version = "0.18", optional = true }
nalgebra    = { workspace = true }
rand        = { workspace = true }
serde       = { workspace = true }
serde_json  = { workspace = true }
sim-core    = { workspace = true }
sim-mjcf    = { workspace = true }
thiserror   = { workspace = true }

[dev-dependencies]
approx      = { workspace = true }
criterion   = { workspace = true }

[[bench]]
name = "bridge_benchmarks"
harness = false

[lints]
workspace = true
```

Carries the `bevy` optional feature forward from `sim-ml-bridge`
(`sim/L0/ml-bridge/Cargo.toml:12`).

### 3.2 Module declarations in `sim-ml-chassis/src/lib.rs`

```rust
pub mod algorithm;
pub mod artifact;
pub mod autograd;
pub mod autograd_layers;
pub mod autograd_policy;
pub mod autograd_value;
pub mod best_tracker;   // NOTE: promoted from `pub(crate)` to `pub`
pub mod competition;
pub mod env;
pub mod error;
pub mod gae;
pub mod linear;
pub mod mlp;
pub mod optimizer;
pub mod policy;
pub mod replay_buffer;
pub mod rollout;
pub mod space;
pub mod stats;          // NEW — holds gaussian_log_prob (§3.6)
pub mod task;
pub mod tensor;
pub mod value;
pub mod vec_env;
```

### 3.3 `pub use` list in `sim-ml-chassis/src/lib.rs`

Copy verbatim from `sim/L0/ml-bridge/src/lib.rs:101-135`, **minus the five
algorithm exports** (`cem`, `reinforce`, `ppo`, `td3`, `sac`):

```rust
pub use algorithm::{Algorithm, EpochMetrics, TrainingBudget};
pub use artifact::{
    ArtifactError, CURRENT_VERSION, NetworkDescriptor, NetworkKind, NetworkSnapshot,
    OptimizerSnapshot, PolicyArtifact, PolicyDescriptor, TrainingCheckpoint, TrainingProvenance,
};
pub use autograd::{Tape, Var};
pub use autograd_layers::{
    Activation, linear_hidden, linear_raw, linear_relu, linear_tanh, mse_loss, mse_loss_batch,
};
pub use autograd_policy::{AutogradPolicy, AutogradStochasticPolicy};
pub use autograd_value::{AutogradQ, AutogradValue};
pub use competition::{Competition, CompetitionResult, RunResult};
pub use env::{Environment, SimEnv, SimEnvBuilder, StepResult};
pub use error::{EnvError, ResetError, SpaceError, TensorError, VecStepError};
pub use gae::compute_gae;
pub use linear::{LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue};
pub use mlp::{MlpPolicy, MlpQ, MlpValue};
pub use optimizer::{Optimizer, OptimizerConfig};
pub use policy::{DifferentiablePolicy, Policy, StochasticPolicy};
pub use replay_buffer::{ReplayBuffer, TransitionBatch};
pub use rollout::{EpisodicRollout, Trajectory, collect_episodic_rollout};
pub use space::{
    ActionSpace, ActionSpaceBuilder, ObsSegment, ObservationSpace, ObservationSpaceBuilder,
};
pub use stats::gaussian_log_prob;   // NEW — relocated from sac.rs, see §3.6
pub use task::{
    TaskConfig, TaskConfigBuilder, obstacle_reaching_6dof, reaching_2dof, reaching_6dof,
};
pub use tensor::{Tensor, TensorSpec};
pub use value::{QFunction, ValueFn, soft_update, soft_update_policy, soft_update_value};
pub use vec_env::{VecEnv, VecEnvBuilder, VecStepResult};
```

Note: `TaskConfig::from_build_fn` already exists upstream (added during the
ml-chassis-refactor study, PR #190). It moves with `task.rs` unchanged —
no new constructor work in this split.

Deliberately **not** re-exported (they move to `sim-rl`):
`Cem`, `CemHyperparams`, `Reinforce`, `ReinforceHyperparams`,
`Ppo`, `PpoHyperparams`, `Sac`, `SacHyperparams`,
`Td3`, `Td3Hyperparams`.

**Changed from the current `sim-ml-bridge` surface:** `gaussian_log_prob`
is now re-exported from `stats` (chassis), not `sac` (sim-rl). The
function body is physically identical; only its owning crate changes.
Rationale in §3.6.

### 3.4 `best_tracker` visibility promotion

In the moved `sim-ml-chassis/src/best_tracker.rs`, module-level:

```rust
// Before (current sim-ml-bridge/src/lib.rs:79):
pub(crate) mod best_tracker;

// After (sim-ml-chassis/src/lib.rs:?):
pub mod best_tracker;
```

Rationale: `sim-rl`'s algorithms all do
`crate::best_tracker::BestTracker::new(...)` internally. After the split,
they become `sim_ml_chassis::best_tracker::BestTracker::new(...)` — which
requires the module to be `pub`. No change to the struct's interface;
`BestTracker::new`, `maybe_update`, `to_checkpoint`, `from_checkpoint`,
`to_artifact` all remain `pub` methods.

Also re-export the type at the chassis root for convenience:

```rust
pub use best_tracker::BestTracker;
```

### 3.5 Intra-crate import rewrites inside moved files

Every file in `sim-ml-chassis/src/` keeps `use crate::...` paths — they
all still point within the same crate after the move. Zero rewrite needed
except one:

- `sim/L0/ml-chassis/src/autograd.rs:77` — doc comment example currently
  says `use sim_ml_bridge::autograd::{Tape, Var};`. Change to
  `use sim_ml_chassis::autograd::{Tape, Var};`.

### 3.6 New `sim-ml-chassis/src/stats.rs`

Single-function module. The body is copy-pasted verbatim from
`sim/L0/ml-bridge/src/sac.rs:226-242`; only the owning crate changes.

```rust
//! Statistics helpers used across policies and algorithms.
//!
//! This module exists to host pure math helpers that belong to neither a
//! specific algorithm nor the autograd tape. They are called by
//! hand-coded code paths (e.g., `sim-rl`'s SAC implementation) and by
//! chassis-level consumers that build RL loops from primitives (e.g.,
//! the `vec-env/sac` visual example). The autograd-tape version of
//! `gaussian_log_prob` lives in [`crate::autograd_layers`] alongside the
//! other `Var`-based ops.

/// Log probability of `action` under a diagonal Gaussian with mean `mu`
/// and per-dimension `log_std`.
///
/// Computes `log N(action | mu, diag(exp(log_std)^2))`.
///
/// # Panics
///
/// Does not panic on its own, but indexes `mu[i]` and `log_std[i]` for
/// `i ∈ [0, action.len())`, so the three slices must be the same length
/// — caller responsibility.
#[must_use]
pub fn gaussian_log_prob(action: &[f64], mu: &[f64], log_std: &[f64]) -> f64 {
    let mut lp = 0.0;
    for i in 0..action.len() {
        let std = log_std[i].exp();
        let var = std * std;
        let diff = action[i] - mu[i];
        lp += 0.5f64.mul_add(
            -(2.0 * std::f64::consts::PI).ln(),
            -0.5 * diff * diff / var - log_std[i],
        );
    }
    lp
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_difference_matches_closed_form() {
        // At action == mu with log_std = 0 (i.e., std=1), log N(0|0,1) per
        // dim is -0.5 * ln(2π).
        let lp = gaussian_log_prob(&[0.0], &[0.0], &[0.0]);
        let expected = -0.5 * (2.0 * std::f64::consts::PI).ln();
        assert!((lp - expected).abs() < 1e-12);
    }

    #[test]
    fn diagonal_independence_sums_per_dim() {
        // Three independent dims should sum.
        let lp = gaussian_log_prob(&[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0], &[0.0, 0.0, 0.0]);
        let per_dim = -0.5 * (2.0 * std::f64::consts::PI).ln();
        assert!((lp - 3.0 * per_dim).abs() < 1e-12);
    }
}
```

**Why this relocation is load-bearing.** `vec-env/sac/src/main.rs:45`
imports `gaussian_log_prob` from `sim_ml_bridge`. Grep-verified: the
visual SAC example builds SAC from chassis primitives (no `Sac` struct
used), so it is a **chassis-only consumer** per §8's classification. If
`gaussian_log_prob` stayed in `sim-rl::sac`, the example would need
`sim-rl` as a dependency for a single math helper — breaking the clean
"chassis for pedagogy, sim-rl for headless baselines" split. Moving the
function to `sim-ml-chassis::stats` makes the example's existing
import path (after the `sim_ml_bridge` → `sim_ml_chassis` rewrite in §8)
work without pulling in `sim-rl`.

`sim-rl`'s `sac.rs` internally also calls `gaussian_log_prob` at lines
`sac.rs:397,519` — those imports also get redirected to
`sim_ml_chassis::stats::gaussian_log_prob`. No behavior change; the
function body is bit-identical.

---

## 4. `sim-rl` — public surface

### 4.1 Cargo.toml

`sim/L0/ml-bridge/Cargo.toml` is renamed and moved to
`sim/L0/rl/Cargo.toml`. Replace its contents with:

```toml
[package]
name = "sim-rl"
description = "Generic RL baselines (CEM, REINFORCE, PPO, TD3, SAC) — the control group for every CortenForge competition"
version.workspace = true
edition.workspace = true
license.workspace = true
repository.workspace = true
authors.workspace = true
rust-version.workspace = true

[dependencies]
nalgebra         = { workspace = true }
rand             = { workspace = true }
serde            = { workspace = true }
serde_json       = { workspace = true }
sim-core         = { workspace = true }
sim-ml-chassis   = { workspace = true }
thiserror        = { workspace = true }

[dev-dependencies]
approx           = { workspace = true }   # for tests/competition.rs assertions
sim-mjcf         = { workspace = true }   # loads MJCF fixtures in tests/{custom_task,competition}.rs

[lints]
workspace = true
```

**No `bevy` feature** — none of the five algorithm files reference
`bevy_ecs` (grep-verified: the feature was carried by
`sim-ml-bridge` for chassis-level consumers, which are now in
`sim-ml-chassis`).

**`sim-mjcf` is a dev-dep only.** The algorithm source files (`cem.rs`,
`ppo.rs`, `td3.rs`, `sac.rs`, `reinforce.rs`) do not reference `sim-mjcf`
directly — they operate on an already-constructed `VecEnv`. Only the
two integration tests under `tests/` load MJCF fixtures.

### 4.2 `sim-rl/src/lib.rs`

```rust
//! # sim-rl
//!
//! Generic RL baselines (CEM, REINFORCE, PPO, TD3, SAC) bolted onto
//! `sim-ml-chassis`. The control group for every CortenForge competition
//! against physics-aware algorithms in `sim-opt`.
//!
//! Re-exports the chassis types that every algorithm consumer also needs,
//! so `use sim_rl::{Cem, Algorithm, VecEnv}` works without reaching into
//! `sim_ml_chassis` directly.

#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod builders;
pub mod cem;
pub mod ppo;
pub mod reinforce;
pub mod sac;
pub mod td3;

// Algorithm exports.
pub use cem::{Cem, CemHyperparams};
pub use ppo::{Ppo, PpoHyperparams};
pub use reinforce::{Reinforce, ReinforceHyperparams};
pub use sac::{Sac, SacHyperparams};
pub use td3::{Td3, Td3Hyperparams};
// Note: `gaussian_log_prob` is NOT re-exported from sim-rl. It lives in
// `sim_ml_chassis::stats` post-relocation (§3.6) and is imported from
// there directly.

// Chassis re-exports — the common import set that algorithm consumers
// need alongside the algorithm structs. Keep this list aligned with
// what d1c/d1d/d2c/persistence actually import, so every site that
// used to say `sim_ml_bridge::{...}` can rewrite to `sim_rl::{...}`
// with a single find-and-replace.
pub use sim_ml_chassis::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
pub use sim_ml_chassis::artifact::{
    PolicyArtifact, TrainingCheckpoint, TrainingProvenance,
};
pub use sim_ml_chassis::env::{Environment, SimEnv, StepResult};
pub use sim_ml_chassis::linear::{LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue};
pub use sim_ml_chassis::mlp::{MlpPolicy, MlpQ, MlpValue};
pub use sim_ml_chassis::optimizer::OptimizerConfig;
pub use sim_ml_chassis::policy::{DifferentiablePolicy, Policy, StochasticPolicy};
pub use sim_ml_chassis::space::{ActionSpace, ObservationSpace};
pub use sim_ml_chassis::task::{TaskConfig, reaching_2dof, reaching_6dof};
pub use sim_ml_chassis::tensor::Tensor;
pub use sim_ml_chassis::value::{QFunction, ValueFn};
pub use sim_ml_chassis::vec_env::VecEnv;
```

**Why `TrainingProvenance` et al. are re-exported.**
`examples/fundamentals/sim-ml/persistence/train-then-replay/src/main.rs:33-36`
imports `TrainingProvenance` from `sim_ml_bridge` alongside `Cem`. Post-rewrite,
that example is a `sim-rl` consumer (it uses `Cem::new`), so `sim-rl` must
re-export `TrainingProvenance` to keep the import single-crate.

### 4.3 Import rewrites inside the algorithm files

**When:** commit 2, while the files still live at `sim/L0/ml-bridge/src/`.
**Not** in commit 3 — commit 3 is a pure rename. See §1.4 for the
sequencing rationale.

Every algorithm file currently uses `use crate::...` for chassis types.
Rewrite to `use sim_ml_chassis::...`:

- `cem.rs`: `crate::{algorithm, artifact, policy, rollout, vec_env, best_tracker}` → `sim_ml_chassis::{algorithm, artifact, policy, rollout, vec_env, best_tracker}`.
- `reinforce.rs`: same substitution, also `crate::optimizer` → `sim_ml_chassis::optimizer`.
- `ppo.rs`: same pattern; includes `crate::gae::compute_gae` → `sim_ml_chassis::gae::compute_gae` and `crate::value::ValueFn` → `sim_ml_chassis::value::ValueFn`.
- `td3.rs`: same pattern; includes `crate::replay_buffer::ReplayBuffer` → `sim_ml_chassis::replay_buffer::ReplayBuffer` and `crate::value::{QFunction, soft_update}` → `sim_ml_chassis::value::{QFunction, soft_update}`.
- `sac.rs`: same pattern as td3.rs.

Mechanical find-and-replace inside the `sim-rl/src/` directory:
`crate::algorithm::` → `sim_ml_chassis::algorithm::`, and similarly for
each of: `artifact`, `best_tracker`, `gae`, `optimizer`, `policy`,
`replay_buffer`, `rollout`, `tensor`, `value`, `vec_env`. Do not rewrite
`crate::cem`, `crate::ppo`, etc. — those remain intra-crate.

### 4.4 `sim-rl/src/builders.rs`

New file. Each builder takes a `&TaskConfig`, reads its `obs_dim`,
`act_dim`, `obs_scale`, and returns a `Box<dyn Algorithm>` constructed
with task-matched hyperparams. The hyperparam defaults match the working
D2c test (`sim/L0/thermostat/tests/d2c_cem_training.rs:274-386`).

```rust
//! Competition-ready builders for the RL baselines.
//!
//! Each builder takes a [`TaskConfig`] and returns a [`Box<dyn Algorithm>`]
//! sized for that task. Hyperparameter defaults are tuned for the D2c
//! stochastic-resonance rematch; override by calling the underlying
//! `::new()` directly in custom tests.

use sim_ml_chassis::algorithm::Algorithm;
use sim_ml_chassis::linear::{LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue};
use sim_ml_chassis::optimizer::OptimizerConfig;
use sim_ml_chassis::task::TaskConfig;

use crate::cem::{Cem, CemHyperparams};
use crate::ppo::{Ppo, PpoHyperparams};
use crate::sac::{Sac, SacHyperparams};
use crate::td3::{Td3, Td3Hyperparams};

const MAX_EPISODE_STEPS: usize = 5_000;

/// CEM with a linear policy. Matches the D2c CEM baseline.
#[must_use]
pub fn build_cem_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    Box::new(Cem::new(
        Box::new(policy),
        CemHyperparams {
            elite_fraction: 0.2,
            noise_std: 2.5,
            noise_decay: 0.98,
            noise_min: 0.1,
            max_episode_steps: MAX_EPISODE_STEPS,
        },
    ))
}

/// PPO with a linear policy + linear value fn. Matches the D2c PPO baseline.
#[must_use]
pub fn build_ppo_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let value_fn = LinearValue::new(task.obs_dim(), task.obs_scale());
    Box::new(Ppo::new(
        Box::new(policy),
        Box::new(value_fn),
        OptimizerConfig::adam(3e-4),
        PpoHyperparams {
            clip_eps: 0.2,
            k_passes: 4,
            gamma: 0.99,
            gae_lambda: 0.95,
            sigma_init: 1.0,
            sigma_decay: 0.995,
            sigma_min: 0.1,
            max_episode_steps: MAX_EPISODE_STEPS,
        },
    ))
}

/// TD3 with linear policy + twin linear Q. Matches the D2c TD3 baseline.
#[must_use]
pub fn build_td3_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let policy        = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let q1            = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let q2            = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_q1     = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_q2     = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    Box::new(Td3::new(
        Box::new(policy),
        Box::new(target_policy),
        Box::new(q1),
        Box::new(q2),
        Box::new(target_q1),
        Box::new(target_q2),
        OptimizerConfig::adam(3e-4),
        Td3Hyperparams {
            gamma: 0.99,
            tau: 0.005,
            policy_noise: 0.2,
            noise_clip: 0.5,
            exploration_noise: 0.5,
            policy_delay: 2,
            batch_size: 256,
            buffer_capacity: 100_000,
            warmup_steps: 1_000,
            max_episode_steps: MAX_EPISODE_STEPS,
        },
    ))
}

/// SAC with linear stochastic policy + twin linear Q. Matches the D2c SAC baseline.
#[must_use]
pub fn build_sac_linear(task: &TaskConfig) -> Box<dyn Algorithm> {
    let policy    = LinearStochasticPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale(), 0.0);
    let q1        = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let q2        = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_q1 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    let target_q2 = LinearQ::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    Box::new(Sac::new(
        Box::new(policy),
        Box::new(q1),
        Box::new(q2),
        Box::new(target_q1),
        Box::new(target_q2),
        OptimizerConfig::adam(3e-4),
        SacHyperparams {
            gamma: 0.99,
            tau: 0.005,
            alpha_init: 0.2,
            auto_alpha: true,
            target_entropy: -(task.act_dim() as f64),
            alpha_lr: 3e-4,
            batch_size: 256,
            buffer_capacity: 100_000,
            warmup_steps: 1_000,
            max_episode_steps: MAX_EPISODE_STEPS,
        },
    ))
}
```

**Note on CEM init.** The D2c test (`d2c_cem_training.rs:278`) calls
`policy.set_params(&[0.0, 0.0, 2.0])` before handing the policy to CEM —
a task-specific prior that pushes the initial temperature mean near the
SR peak. Do **not** bake this into `build_cem_linear`; it's overfitting
to SR. The rematch test (§7) can apply the same prior inline before
calling the builder's output if parity with D2c is required.

**MLP / autograd builders are deferred** — the spec's decision is "linear
RL vs physics-aware first; MLP follow-up later." Do not write
`build_ppo_mlp` / `build_sac_autograd` in this PR.

---

## 5. `sim-opt` — public surface and SA skeleton

### 5.1 Cargo.toml

```toml
[package]
name = "sim-opt"
description = "Physics-aware optimization algorithms (SA, PT, CMA-ES, MH) — the custom-skate family for thermodynamic computing"
version.workspace = true
edition.workspace = true
license.workspace = true
repository.workspace = true
authors.workspace = true
rust-version.workspace = true

[dependencies]
nalgebra        = { workspace = true }
rand            = { workspace = true }
serde           = { workspace = true }
serde_json      = { workspace = true }
sim-core        = { workspace = true }
sim-ml-chassis  = { workspace = true }
sim-thermostat  = { workspace = true }
thiserror       = { workspace = true }

[dev-dependencies]
approx          = { workspace = true }
sim-mjcf        = { workspace = true }
sim-rl          = { workspace = true }   # baseline comparisons only

[lints]
workspace = true
```

### 5.2 `sim-opt/src/lib.rs`

```rust
//! # sim-opt
//!
//! Physics-aware optimization algorithms for CortenForge. Peers with
//! `sim-rl` — both bolt onto `sim-ml-chassis`. `sim-opt` is where
//! algorithms whose structure matches the physics of thermodynamic
//! computing live: Simulated Annealing, Parallel Tempering, CMA-ES,
//! Metropolis-Hastings.

#![deny(clippy::unwrap_used, clippy::expect_used)]

pub mod annealing;
pub mod builders;
pub mod cma_es;
pub mod metropolis;
pub mod tempering;

pub use annealing::{AnnealingHyperparams, CoolingSchedule, SimulatedAnnealing};
pub use cma_es::{CmaEs, CmaEsHyperparams};
pub use metropolis::{MetropolisHastings, MhHyperparams};
pub use tempering::{ParallelTempering, TemperingHyperparams};

// Re-export the chassis `Algorithm` trait so downstream users can import
// everything through `sim_opt::` for symmetry with `sim_rl::`.
pub use sim_ml_chassis::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
pub use sim_ml_chassis::task::TaskConfig;
pub use sim_ml_chassis::vec_env::VecEnv;
```

### 5.3 `sim-opt/src/annealing.rs` — full skeleton

This is the one file that gets a real implementation in this PR. The
skeleton below compiles and satisfies `Algorithm`; the body of the inner
loop is where the author does the real work.

```rust
//! Simulated Annealing (Kirkpatrick, Gelatt, Vecchi — Science 1983).
//!
//! Each epoch: propose a perturbation of the current candidate params,
//! evaluate fitness via episodic rollout on `VecEnv`, accept or reject via
//! the Metropolis criterion at the current temperature, then step the
//! cooling schedule.
//!
//! Fits `sim_ml_chassis::Algorithm` as-is — SA treats the policy parameter
//! vector as its state and the mean episodic return as its objective.

use std::collections::BTreeMap;
use std::time::Instant;

use rand::SeedableRng;
use rand::rngs::StdRng;

use sim_ml_chassis::algorithm::{Algorithm, EpochMetrics, TrainingBudget};
use sim_ml_chassis::artifact::{PolicyArtifact, TrainingCheckpoint};
use sim_ml_chassis::best_tracker::BestTracker;
use sim_ml_chassis::policy::Policy;
use sim_ml_chassis::rollout::collect_episodic_rollout;
use sim_ml_chassis::vec_env::VecEnv;

// ── Cooling schedules ────────────────────────────────────────────────────

/// Temperature schedule for the annealing loop.
///
/// `Geometric` is the canonical choice from Kirkpatrick et al. `Linear`
/// is offered for debugging and short runs.
#[derive(Debug, Clone, Copy)]
pub enum CoolingSchedule {
    /// `T(t+1) = T(t) * factor`. Classic Kirkpatrick.
    Geometric { factor: f64 },
    /// `T(t+1) = max(min, T(t) - step)`. For linear-cooldown debugging.
    Linear { step: f64, min: f64 },
}

// ── Hyperparameters ──────────────────────────────────────────────────────

/// Simulated Annealing hyperparameters.
#[derive(Debug, Clone, Copy)]
pub struct AnnealingHyperparams {
    /// Initial temperature (in units of fitness per parameter perturbation).
    pub t_init: f64,
    /// Minimum temperature floor. Below this the algorithm becomes greedy.
    pub t_min: f64,
    /// Cooling schedule applied once per epoch.
    pub schedule: CoolingSchedule,
    /// Standard deviation of Gaussian proposal perturbations. One shared σ
    /// across all policy parameters; tune if parameters have different scales.
    pub proposal_std: f64,
    /// Maximum environment steps per episode (evaluation budget per candidate).
    pub max_episode_steps: usize,
}

// ── SimulatedAnnealing ───────────────────────────────────────────────────

/// Simulated Annealing algorithm.
///
/// # Parts
///
/// - [`Policy`] — SA operates on the policy's parameter vector directly.
///   No gradients, no autograd. Any `Box<dyn Policy>` works.
///
/// # Constructor
///
/// ```ignore
/// let sa = SimulatedAnnealing::new(
///     Box::new(LinearPolicy::new(obs_dim, act_dim, &obs_scale)),
///     AnnealingHyperparams {
///         t_init: 1.0,
///         t_min: 0.01,
///         schedule: CoolingSchedule::Geometric { factor: 0.95 },
///         proposal_std: 0.3,
///         max_episode_steps: 5_000,
///     },
/// );
/// ```
pub struct SimulatedAnnealing {
    policy: Box<dyn Policy>,
    hyperparams: AnnealingHyperparams,
    /// Current temperature. Decayed by `hyperparams.schedule` each epoch.
    temperature: f64,
    /// Best-epoch policy snapshot.
    best: BestTracker,
}

impl SimulatedAnnealing {
    /// Create a new Simulated Annealing instance.
    #[must_use]
    pub fn new(policy: Box<dyn Policy>, hyperparams: AnnealingHyperparams) -> Self {
        let temperature = hyperparams.t_init;
        let best = BestTracker::new(policy.params());
        Self { policy, hyperparams, temperature, best }
    }

    /// Advance the cooling schedule one step.
    fn cool(&mut self) {
        self.temperature = match self.hyperparams.schedule {
            CoolingSchedule::Geometric { factor } => {
                (self.temperature * factor).max(self.hyperparams.t_min)
            }
            CoolingSchedule::Linear { step, min } => {
                (self.temperature - step).max(min.max(self.hyperparams.t_min))
            }
        };
    }
}

impl Algorithm for SimulatedAnnealing {
    fn name(&self) -> &'static str { "SA" }

    fn train(
        &mut self,
        env: &mut VecEnv,
        budget: TrainingBudget,
        seed: u64,
        on_epoch: &dyn Fn(&EpochMetrics),
    ) -> Vec<EpochMetrics> {
        let mut rng = StdRng::seed_from_u64(seed);
        let n_epochs = match budget {
            TrainingBudget::Epochs(n) => n,
            // SA is inherently epoch-based (one candidate per epoch) and
            // each epoch runs TWO rollouts (curr + cand — see the
            // "stochastic env handling" note below). Map Steps → Epochs
            // by dividing by `2 * max_episode_steps`, with a floor of 1.
            TrainingBudget::Steps(s) => {
                (s / (2 * self.hyperparams.max_episode_steps).max(1)).max(1)
            }
        };
        let mut metrics = Vec::with_capacity(n_epochs);

        // Execution-session inner loop. Every design decision is
        // written down — no hidden choices.
        //
        // ── Fitness metric ──────────────────────────────────────────
        // Same as CEM (`cem.rs:177-184` on the current tree): per-
        // trajectory mean reward `sum(rewards) / len.max(1)`, averaged
        // over all `n_envs` trajectories. SR episodes run for 5 000
        // steps, so per-step mean keeps the fitness on the same scale
        // as the raw Langevin reward signal. **Do not** use `sum` — it
        // rewards longer episodes, which skews SA's accept/reject rule
        // when episodes truncate at slightly different times.
        //
        // ── Stochastic-env handling ─────────────────────────────────
        // SR is Langevin-driven; every rollout is noisy. SA
        // **re-evaluates curr every epoch** rather than caching its
        // score, so each Metropolis comparison is between two freshly
        // drawn samples of the same process. Cost: 2× rollouts/epoch.
        // Benefit: the accept/reject decision is unbiased against
        // whichever candidate happened to draw a lucky early sample.
        // The n_epochs mapping above accounts for this.
        //
        // ── Borrow-checker shape ────────────────────────────────────
        // `Policy::set_params` is `&mut self`; `Policy::forward` is
        // `&self`. The `act_fn` closure passed to
        // `collect_episodic_rollout` only needs `forward`, so
        // `set_params` happens *outside* the closure. CEM nests
        // set_params *inside* the closure because it has one candidate
        // per env; SA has one candidate per epoch, so it doesn't.
        //
        // ── Imports to add at the top of this file ──────────────────
        //   use sim_ml_chassis::rollout::{Trajectory, EpisodicRollout};
        // (already importing `collect_episodic_rollout` — Trajectory is
        // needed for the `.len()` method reference below.)
        //
        // ── Skeleton ────────────────────────────────────────────────
        //
        //   fn mean_fitness(rollout: &EpisodicRollout) -> f64 {
        //       let n = rollout.trajectories.len().max(1) as f64;
        //       rollout
        //           .trajectories
        //           .iter()
        //           .map(|t| t.rewards.iter().sum::<f64>()
        //                    / t.len().max(1) as f64)
        //           .sum::<f64>() / n
        //   }
        //
        //   for epoch in 0..n_epochs {
        //       let t0 = Instant::now();
        //       let curr: Vec<f64> = self.policy.params().to_vec();
        //
        //       // Evaluate curr (fresh sample, not a cache).
        //       self.policy.set_params(&curr);
        //       let rollout_curr = collect_episodic_rollout(
        //           env,
        //           &mut |_env_idx, obs| self.policy.forward(obs),
        //           self.hyperparams.max_episode_steps,
        //       );
        //       let score_curr = mean_fitness(&rollout_curr);
        //
        //       // Propose: cand = curr + Normal(0, proposal_std).
        //       let proposal_std = self.hyperparams.proposal_std;
        //       let cand: Vec<f64> = curr
        //           .iter()
        //           .map(|&p| proposal_std.mul_add(randn(&mut rng), p))
        //           .collect();
        //       self.policy.set_params(&cand);
        //       let rollout_cand = collect_episodic_rollout(
        //           env,
        //           &mut |_env_idx, obs| self.policy.forward(obs),
        //           self.hyperparams.max_episode_steps,
        //       );
        //       let score_cand = mean_fitness(&rollout_cand);
        //
        //       // Metropolis acceptance (max problem — higher = better).
        //       let delta = score_cand - score_curr;
        //       let accepted = delta >= 0.0
        //           || rng.random::<f64>() < (delta / self.temperature).exp();
        //       if !accepted {
        //           self.policy.set_params(&curr); // restore
        //       }
        //
        //       let score_after = if accepted { score_cand } else { score_curr };
        //       self.best.maybe_update(
        //           epoch, score_after, self.policy.params(),
        //       );
        //
        //       let total_steps: usize = rollout_curr
        //           .trajectories
        //           .iter()
        //           .chain(rollout_cand.trajectories.iter())
        //           .map(Trajectory::len)
        //           .sum();
        //       let done_count = rollout_cand
        //           .trajectories
        //           .iter()
        //           .filter(|t| t.done)
        //           .count();
        //
        //       self.cool();
        //
        //       let mut extra = BTreeMap::new();
        //       extra.insert("temperature".into(), self.temperature);
        //       extra.insert(
        //           "accepted".into(), if accepted { 1.0 } else { 0.0 },
        //       );
        //       extra.insert("delta".into(), delta);
        //
        //       let em = EpochMetrics {
        //           epoch,
        //           mean_reward: score_after,
        //           done_count,
        //           total_steps,
        //           wall_time_ms: t0.elapsed().as_millis() as u64,
        //           extra,
        //       };
        //       on_epoch(&em);
        //       metrics.push(em);
        //   }
        //
        // ── randn helper ────────────────────────────────────────────
        // Copy the Box-Muller at `cem.rs:115-119` verbatim into this
        // file (four lines, plus doc comment). Do not re-export it
        // across crates — it is private in `sim-rl` by design.
        let _ = (env, &mut rng, on_epoch, &mut metrics, n_epochs);
        todo!("SA inner loop — fill in per the skeleton above")
    }

    fn policy_artifact(&self) -> PolicyArtifact {
        PolicyArtifact::from_policy(&*self.policy)
    }

    fn best_artifact(&self) -> PolicyArtifact {
        self.best.to_artifact(self.policy.descriptor())
    }

    fn checkpoint(&self) -> TrainingCheckpoint {
        let (best_params, best_reward, best_epoch) = self.best.to_checkpoint();
        let mut state = BTreeMap::new();
        state.insert("temperature".into(), self.temperature);
        TrainingCheckpoint {
            algorithm_name: "SA".into(),
            policy_artifact: self.policy_artifact(),
            critics: vec![],
            optimizer_states: vec![],
            algorithm_state: state,
            best_params: Some(best_params),
            best_reward,
            best_epoch,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geometric_schedule_cools_and_clamps() {
        // Construct with a dummy zero-param policy so we can test cooling
        // in isolation without touching an env.
        let hyp = AnnealingHyperparams {
            t_init: 1.0,
            t_min: 0.1,
            schedule: CoolingSchedule::Geometric { factor: 0.5 },
            proposal_std: 0.1,
            max_episode_steps: 10,
        };
        // Use LinearPolicy with 1-dim obs, 1-dim act — dev-dep on sim-rl
        // for the type, or re-export from chassis (already done).
        let policy = Box::new(sim_ml_chassis::LinearPolicy::new(1, 1, &[1.0]));
        let mut sa = SimulatedAnnealing::new(policy, hyp);

        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.5);
        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.25);
        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.125);
        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.1); // clamped
        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.1); // stays
    }

    #[test]
    fn linear_schedule_cools_and_clamps() {
        let hyp = AnnealingHyperparams {
            t_init: 1.0,
            t_min: 0.05,
            schedule: CoolingSchedule::Linear { step: 0.3, min: 0.05 },
            proposal_std: 0.1,
            max_episode_steps: 10,
        };
        let policy = Box::new(sim_ml_chassis::LinearPolicy::new(1, 1, &[1.0]));
        let mut sa = SimulatedAnnealing::new(policy, hyp);

        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.7);
        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.4);
        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.1);
        sa.cool(); approx::assert_relative_eq!(sa.temperature, 0.05); // clamped
    }
}
```

### 5.4 Stub files

`tempering.rs`, `cma_es.rs`, `metropolis.rs` each contain just the type
definitions needed for `sim-opt/src/lib.rs` to compile. Every method body
is `todo!("sim-opt phase 2")`. Example:

```rust
//! Parallel Tempering (Geyer 1991; Hukushima & Nemoto 1996). Stub.

pub struct ParallelTempering { /* fields TBD */ }

#[derive(Debug, Clone, Copy)]
pub struct TemperingHyperparams {
    pub n_replicas: usize,
    pub t_min: f64,
    pub t_max: f64,
    pub swap_interval: usize,
    pub max_episode_steps: usize,
}

impl ParallelTempering {
    #[must_use]
    pub fn new(_hyp: TemperingHyperparams) -> Self { todo!("sim-opt phase 2") }
}
```

These stubs exist so `sim-opt` has a coherent public surface after the
first PR. They do not implement `Algorithm` yet; they are not exposed
via `sim_opt::builders::*` until phase 2.

### 5.5 `sim-opt/src/builders.rs`

```rust
//! Competition-ready builders for the sim-opt algorithm family.

use sim_ml_chassis::algorithm::Algorithm;
use sim_ml_chassis::linear::LinearPolicy;
use sim_ml_chassis::task::TaskConfig;

use crate::annealing::{AnnealingHyperparams, CoolingSchedule, SimulatedAnnealing};

const MAX_EPISODE_STEPS: usize = 5_000;

/// Simulated Annealing with a linear policy. The SA baseline for the D2c rematch.
#[must_use]
pub fn build_simulated_annealing(task: &TaskConfig) -> Box<dyn Algorithm> {
    let policy = LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale());
    Box::new(SimulatedAnnealing::new(
        Box::new(policy),
        AnnealingHyperparams {
            t_init: 1.0,
            t_min: 0.01,
            schedule: CoolingSchedule::Geometric { factor: 0.95 },
            proposal_std: 0.5,
            max_episode_steps: MAX_EPISODE_STEPS,
        },
    ))
}
```

PT and CMA-ES builders are **not** written in this PR.

---

## 6. SR task packaging — deferred

The construction spec originally proposed extracting `stochastic_resonance()`
into `sim_thermostat::tasks`. Ch 42 §6 sub-decision (f) intentionally kept the
SR MJCF + constants duplicated in the three rematch fixtures under
`sim/L0/opt/tests/`. The chassis/rl split does not revisit that decision. If a
future PR wants SR-as-`TaskConfig`, see `project_thermo_rl_loop_vision.md` for
the motivation.

---

## 7. D2c rematch — `sim/L0/opt/tests/d2c_rematch.rs`

New integration test. Mirrors the D2c experiment structure (32 envs, 100
epochs, 5000-step episodes) but uses `Competition::run` to pool RL
baselines against Simulated Annealing. Runs only in release mode.

```rust
//! D2c rematch — Simulated Annealing vs linear-RL baselines on
//! stochastic resonance.
//!
//! Protocol: `Competition::run` with the same 32-env / 100-epoch / 5000-step
//! budget the original D2c tests used. Evaluation is the `best_reward`
//! reported by the Competition runner (max mean-reward across epochs).
//!
//! Gate: SA's best reward must match or exceed the best of the three
//! linear-RL baselines (CEM, PPO, TD3). SAC is excluded because its D2c
//! result (Gate A FAIL, `kT=-0.78`) has no reasonable way to be a "baseline"
//! — include it later if we want the full ranking.
//!
//! Expected runtime: ~40-60 minutes on release. Gated behind `#[ignore]`
//! unless explicitly requested.
//!
//! Spec: `docs/thermo_computing/01_vision/physics_aware_ml_construction.md` §7

#![allow(clippy::unwrap_used, clippy::expect_used)]

use sim_ml_chassis::algorithm::{Algorithm, TrainingBudget};
use sim_ml_chassis::competition::Competition;
use sim_ml_chassis::task::TaskConfig;

use sim_opt::builders::build_simulated_annealing;
use sim_rl::builders::{build_cem_linear, build_ppo_linear, build_td3_linear};
use sim_thermostat::tasks::stochastic_resonance;

const N_ENVS: usize = 32;
const N_EPOCHS: usize = 100;
const SEED: u64 = 20_260_412;

#[test]
#[ignore = "requires --release (~45 min)"]
fn d2c_rematch_sa_vs_linear_rl() {
    let tasks = [stochastic_resonance()];
    let comp = Competition::new_verbose(N_ENVS, TrainingBudget::Epochs(N_EPOCHS), SEED);

    let builders: &[&dyn Fn(&TaskConfig) -> Box<dyn Algorithm>] = &[
        &build_cem_linear,
        &build_ppo_linear,
        &build_td3_linear,
        &build_simulated_annealing,
    ];

    let result = comp.run(&tasks, builders).expect("competition.run");
    result.print_ranked("stochastic-resonance", "D2c rematch");
    for run in &result.runs {
        run.assert_finite();
    }

    let sa  = result.find("stochastic-resonance", "SA").expect("SA result");
    let cem = result.find("stochastic-resonance", "CEM").expect("CEM result");
    let ppo = result.find("stochastic-resonance", "PPO").expect("PPO result");
    let td3 = result.find("stochastic-resonance", "TD3").expect("TD3 result");

    let sa_best  = sa.best_reward().expect("SA best");
    let cem_best = cem.best_reward().expect("CEM best");
    let ppo_best = ppo.best_reward().expect("PPO best");
    let td3_best = td3.best_reward().expect("TD3 best");

    let best_rl = cem_best.max(ppo_best).max(td3_best);

    eprintln!("D2c rematch:");
    eprintln!("  SA  best reward = {sa_best:>10.4}");
    eprintln!("  CEM best reward = {cem_best:>10.4}");
    eprintln!("  PPO best reward = {ppo_best:>10.4}");
    eprintln!("  TD3 best reward = {td3_best:>10.4}");
    eprintln!("  best_linear_rl  = {best_rl:>10.4}");

    // Primary gate: SA >= best linear-RL baseline.
    // Weak form — a tie is acceptable for a first rematch. If SA barely
    // matches, open an issue to investigate proposal-std / schedule tuning
    // before claiming the physics-aware pivot is vindicated.
    assert!(
        sa_best >= best_rl,
        "D2c rematch FAILED: SA best ({sa_best:.4}) < best linear-RL ({best_rl:.4}). \
         Physics-aware did not beat matched-complexity RL on SR."
    );
}
```

**Acceptance criterion.** The test passes if `SA.best_reward ≥
max(CEM, PPO, TD3).best_reward`. The test does **not** try to replicate
the Gate A t-statistic from the original D2c — that's a downstream
analysis the author runs after the competition completes. Keeping the
gate simple minimises the number of things that can flake.

**What a failing rematch looks like.** SA loses. The user memory
`project_d2_sr_findings.md` notes that SR has a broad, flat fitness
landscape that CEM couldn't navigate within. SA with a Gaussian proposal
is structurally similar to CEM's perturbation — it may hit the same
ceiling. If that happens, the PR still lands (the infrastructure is
correct), the test stays `#[ignore]` and green, and the next session is
"tune SA proposal_std / schedule, or skip to PT." Don't block the PR on
the rematch result.

---

## 8. Import rewrite table (step 4 of execution)

Single pass across the tree. `sim_ml_bridge::` disappears; each use site
rewrites to **one** of the two new paths.

**Uniform rewrite rule.** *If the site imports any of `Cem`, `Reinforce`,
`Ppo`, `Sac`, `Td3`, their `*Hyperparams` structs, `TrainingProvenance`,
`PolicyArtifact`, or `TrainingCheckpoint`, rewrite to `sim_rl::`.
Otherwise rewrite to `sim_ml_chassis::`.* The sim-rl re-exports from §4.2
cover every chassis type that actual algorithm consumers also pull in,
so a "mixed" site can always land on `sim_rl::` alone.

**Critical correction from the review pass:** the five "algorithm"
examples (`vec-env/{ppo,sac,td3,reinforce}`, `6dof/cem-linear`) do **not**
import the `Cem`/`Ppo`/`Sac`/`Td3`/`Reinforce` structs — they build RL
loops from chassis primitives (`compute_gae`, `ReplayBuffer`,
`gaussian_log_prob`, etc.) per the design comment at
`sim/L0/ml-bridge/src/algorithm.rs:6-11`. They are **chassis-only
consumers.** Grep-verified: the only example crate that uses an
algorithm struct is `persistence/train-then-replay` (imports `Cem`).

### 8.1 Chassis-only sites (rewrite to `sim_ml_chassis::`)

| File | Actual imports (verified) | Target |
|---|---|---|
| `examples/fundamentals/sim-ml/spaces/obs-rich/src/main.rs` | `ObservationSpace` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/spaces/obs-extract/src/main.rs` | `ObservationSpace` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/spaces/act-inject/src/main.rs` | `ActionSpace, Tensor` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/spaces/act-clamping/src/main.rs` | `ActionSpace, Tensor` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/spaces/stress-test/src/main.rs` | `ActionSpace, ObservationSpace, SpaceError, Tensor` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/sim-env/episode-loop/src/main.rs` | `ActionSpace, Environment, ObservationSpace, SimEnv, Tensor` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/sim-env/on-reset/src/main.rs` | same | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/sim-env/stress-test/src/main.rs` | same | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/sim-env/sub-stepping/src/main.rs` | same | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/parallel-step/src/main.rs` | `ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/terminal-obs/src/main.rs` | `ActionSpace, ObservationSpace, Tensor, VecEnv` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/stress-test/src/main.rs` | `ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv, VecStepError` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/auto-reset/src/main.rs` | `LinearPolicy, Policy, Tensor, VecEnv, reaching_2dof` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/reinforce/src/main.rs` | `DifferentiablePolicy, LinearPolicy, Optimizer, OptimizerConfig, Policy, Tensor, Trajectory, VecEnv, reaching_2dof` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/ppo/src/main.rs` | `DifferentiablePolicy, LinearPolicy, LinearValue, Optimizer, OptimizerConfig, Policy, Tensor, Trajectory, ValueFn, VecEnv, compute_gae, reaching_2dof` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/td3/src/main.rs` | `DifferentiablePolicy, LinearPolicy, LinearQ, Optimizer, OptimizerConfig, Policy, QFunction, ReplayBuffer, Tensor, VecEnv, reaching_2dof, soft_update, soft_update_policy` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/vec-env/sac/src/main.rs` | `LinearQ, LinearStochasticPolicy, Optimizer, OptimizerConfig, Policy, QFunction, ReplayBuffer, StochasticPolicy, Tensor, VecEnv, gaussian_log_prob, reaching_2dof, soft_update` | `sim_ml_chassis::` (works because `gaussian_log_prob` moved to chassis per §3.6) |
| `examples/fundamentals/sim-ml/6dof/cem-linear/src/main.rs` | `ActionSpace, LinearPolicy, ObservationSpace, Policy, Tensor, VecEnv, reaching_6dof` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/6dof/stress-test/src/main.rs` | `Activation, AutogradStochasticPolicy, Environment, LinearPolicy, MlpPolicy, Policy, SimEnv, StochasticPolicy, Tensor, VecEnv, reaching_6dof` | `sim_ml_chassis::` |
| `examples/fundamentals/sim-ml/shared/src/lib.rs` | `TaskConfig, VecEnv` | `sim_ml_chassis::` |
| `sim/L0/thermostat/tests/d2b_stochastic_resonance_baselines.rs` | `ActionSpace, Environment, ObservationSpace, SimEnv, Tensor, VecEnv, rollout::collect_episodic_rollout` | `sim_ml_chassis::` |

### 8.2 sim-rl sites (rewrite to `sim_rl::`)

| File | Actual imports (verified) | Target |
|---|---|---|
| `examples/fundamentals/sim-ml/persistence/train-then-replay/src/main.rs` | `Algorithm, Cem, CemHyperparams, LinearPolicy, Policy, Tensor, TrainingBudget, TrainingProvenance, VecEnv, reaching_2dof` | `sim_rl::` (re-exports cover `TrainingProvenance`) |
| `sim/L0/thermostat/tests/d1c_cem_training.rs` | `ActionSpace, Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, ObservationSpace, Policy, SimEnv, Tensor, TrainingBudget, VecEnv` | `sim_rl::` |
| `sim/L0/thermostat/tests/d1d_reinforce_comparison.rs` | `ActionSpace, Algorithm, Environment, LinearPolicy, ObservationSpace, OptimizerConfig, Policy, Reinforce, ReinforceHyperparams, SimEnv, Tensor, TrainingBudget, VecEnv` | `sim_rl::` |
| `sim/L0/thermostat/tests/d2c_cem_training.rs` | `ActionSpace, Algorithm, Cem, CemHyperparams, Environment, LinearPolicy, LinearQ, LinearStochasticPolicy, LinearValue, ObservationSpace, OptimizerConfig, Policy, Ppo, PpoHyperparams, Sac, SacHyperparams, SimEnv, Td3, Td3Hyperparams, Tensor, TrainingBudget, VecEnv` | `sim_rl::` |
| `sim/L0/rl/tests/custom_task.rs` (moved from `sim/L0/ml-bridge/tests/`) | `ActionSpace, Algorithm, Cem, CemHyperparams, LinearPolicy, ObservationSpace, TaskConfig, TrainingBudget` | `sim_rl::` |
| `sim/L0/rl/tests/competition.rs` (moved from `sim/L0/ml-bridge/tests/`) | (Phases 3+6 Cem/Ppo/Sac/Td3 comparison imports) | `sim_rl::` |

### 8.3 Miscellaneous rewrites

- **`sim/L0/ml-chassis/src/autograd.rs:77`** — doc comment example says
  `use sim_ml_bridge::autograd::{Tape, Var};`. Change to
  `use sim_ml_chassis::autograd::{Tape, Var};`.
- **`sim/L0/ml-chassis/src/autograd_policy.rs:20`** — internal
  `use crate::autograd_layers::{..., gaussian_log_prob, ...};` (Var version).
  This is the autograd-tape `gaussian_log_prob`, not the f64 version
  moved to `stats.rs`. Leave untouched — still in `autograd_layers`.
- **Docs grep.** `grep -rn 'sim_ml_bridge\|sim-ml-bridge' docs/
  examples/` after the code rewrites, and fix whatever turns up.
  `examples/fundamentals/sim-ml/ML_COMPETITION_SPEC.md` has **four
  prose mentions** of the `sim-ml-bridge` crate name (line 4 Crate
  header; line 1020 "Phase 1: Core abstractions (sim-ml-bridge)";
  line 1057 "In sim-ml-bridge, not in examples…"; line 1106 "shared
  building blocks from sim-ml-bridge") and **zero `use sim_ml_bridge::`
  code examples** — grep-verified on 2026-04-12. The right treatment
  is a single banner at the top of that doc, not in-place rewrites:
  the document is a historical phase-planning record from when the
  crate was genuinely called `sim-ml-bridge`, and rewriting the prose
  would falsify the history. The execution session should add,
  above the existing "Status" line:
  ```markdown
  > **Pre-split historical record.** This spec was written when the
  > ML layer was a single `sim-ml-bridge` crate. It has since been
  > split into `sim-ml-chassis` + `sim-rl` + `sim-opt` — see
  > `docs/thermo_computing/01_vision/physics_aware_ml_construction.md`
  > for the current shape. The `sim-ml-bridge` references below are
  > historical; do not use them as import paths.
  ```
  No other edits. Low priority; the banner is a one-line commit and
  can be folded into the step 4 find-and-replace commit.

### 8.4 sim-opt sites

sim-opt currently imports `sim-ml-bridge` as its chassis-in-disguise. The
split forces a rewrite of every source file and test fixture in the crate.
These rewrites land in commit 5 of §10, not commit 4, because the diff is
mechanically distinct (sim-opt is its own crate re-homing, not an external
consumer) and benefits from its own grading checkpoint.

| File | Actual imports (verified 2026-04-15) | Target |
|---|---|---|
| `sim/L0/opt/Cargo.toml` | `sim-ml-bridge` dep | `sim-ml-chassis` regular + `sim-rl` dev-dep (the rematch fixtures need `Cem` and `CemHyperparams`) |
| `sim/L0/opt/src/lib.rs` | module doc mentions `sim-ml-bridge` at lines 7, 27, 29, 32 | rewrite prose to `sim-ml-chassis` / `sim-rl` as appropriate |
| `sim/L0/opt/src/algorithm.rs:20` | `Algorithm, ArtifactError, CURRENT_VERSION, EpochMetrics, Policy, PolicyArtifact, TrainingBudget, TrainingCheckpoint, VecEnv, collect_episodic_rollout` | `sim_ml_chassis::` (all chassis types) |
| `sim/L0/opt/src/algorithm.rs:375` (test mod) | `LinearPolicy, reaching_2dof` | `sim_ml_chassis::` |
| `sim/L0/opt/src/richer_sa.rs:50,492,511` | same chassis-type shape as algorithm.rs | `sim_ml_chassis::` |
| `sim/L0/opt/src/parallel_tempering.rs:64,484,499` | same shape | `sim_ml_chassis::` |
| `sim/L0/opt/src/analysis.rs:17` | `Algorithm, Competition, CompetitionResult, EnvError, TaskConfig` | `sim_ml_chassis::` |
| `sim/L0/opt/src/analysis.rs:634` (test mod) | mixed — primitives + algorithm structs | chassis for primitives, `sim_rl::` for any algorithm struct it references |
| `sim/L0/opt/tests/d2c_sr_rematch.rs:42` | `ActionSpace, Algorithm, Cem, CemHyperparams, Competition, LinearPolicy, ObservationSpace, Policy, TaskConfig, TrainingBudget, VecEnv` | `sim_rl::` (Cem + CemHyperparams force the sim-rl route; sim-rl's re-exports cover the chassis types in the same import) |
| `sim/L0/opt/tests/d2c_sr_rematch_richer_sa.rs:61` | same shape | `sim_rl::` |
| `sim/L0/opt/tests/d2c_sr_rematch_pt.rs:52` | same shape | `sim_rl::` |

Verification: `cargo xtask grade sim-opt` = A; `cargo test -p sim-opt --release`
on non-ignored tests stays green; the three `#[ignore]`'d rematch fixtures
still compile (they don't need to be re-run — the study is closed).

---

## 9. Workspace `Cargo.toml` edits

### 9.1 Workspace members (step 1-3 of execution)

In `Cargo.toml`, `[workspace] members = [ ... ]`:

- Line 304 (`"sim/L0/ml-bridge",`) → `"sim/L0/rl",`.
- Insert new entries before/after as preferred:
  ```
  "sim/L0/ml-chassis",
  "sim/L0/rl",
  "sim/L0/opt",
  ```

### 9.2 Workspace dependencies

In `[workspace.dependencies]`:

- Delete `sim-ml-bridge = { path = "sim/L0/ml-bridge" }` (currently
  line 480).
- Insert in its place:
  ```toml
  sim-ml-chassis = { path = "sim/L0/ml-chassis" }
  sim-rl         = { path = "sim/L0/rl" }
  sim-opt        = { path = "sim/L0/opt" }
  ```

Adjacent entries (`sim-core` at 476, `sim-mjcf` at 477, `sim-thermostat`
at 481) are unchanged. `approx = "0.5"` at line 442 is already in
`[workspace.dependencies]`, so every new crate's `approx = { workspace
= true }` dev-dep resolves without further edits.

---

## 10. Execution commit sequence (one PR, multiple commits)

Each numbered step is one commit. Run the listed grading command before
moving on; if grade drops from A, fix before proceeding (commit may be
amended *within its own step*, never across steps).

| # | Commit message | What it does | Verification |
|---|---|---|---|
| 1 | `feat(sim-ml-chassis): scaffold empty crate` | New `sim/L0/ml-chassis/{Cargo.toml, src/lib.rs}` (empty lib). Workspace member + dep alias. | `cargo build -p sim-ml-chassis` green |
| 2 | `refactor(sim-ml-chassis): move primitives from sim-ml-bridge` | Move the 23 files from §1.2 plus `benches/bridge_benchmarks.rs`. Promote `best_tracker` to `pub`. Create `sim-ml-chassis/src/stats.rs` per §3.6 (with the two unit tests inside it). Update `sim-ml-chassis/src/lib.rs` per §3.2 and §3.3 (module + `pub use` for `stats::gaussian_log_prob`). Fix the `autograd.rs:77` docstring per §3.5. `TaskConfig::from_build_fn` already exists upstream and moves with `task.rs` unchanged — no new constructor code. **`tests/custom_task.rs` and `tests/competition.rs` do NOT move here** — they stay with the soon-to-be-renamed sim-rl. **sim-ml-bridge still exists** at this commit — the algorithm files are all that's left. Update `sim-ml-bridge/src/lib.rs` to drop the moved modules and keep only `cem`, `ppo`, `reinforce`, `sac`, `td3` with `use sim_ml_chassis::*` as needed. Also delete the local `pub fn gaussian_log_prob` in `sim-ml-bridge/src/sac.rs:226-242` and redirect its two internal callers at `sac.rs:397,519` to `sim_ml_chassis::stats::gaussian_log_prob`. | `cargo build -p sim-ml-chassis` green; `cargo test -p sim-ml-chassis stats::tests` green; `cargo build -p sim-ml-bridge` green; `cargo xtask grade sim-ml-chassis` = A |
| 3 | `refactor(sim-rl): rename sim-ml-bridge and re-export chassis types` | Rename `sim/L0/ml-bridge/` → `sim/L0/rl/`. Replace the Cargo.toml contents per §4.1 (drops the transitional `sim-ml-chassis` dep line from commit 2, adds `approx` + `sim-mjcf` dev-deps, changes package name to `sim-rl`). Rewrite `src/lib.rs` per §4.2 (re-exports include `TrainingProvenance`, `PolicyArtifact`, `TrainingCheckpoint`). Create `builders.rs` per §4.4. The two integration tests travel with the rename (they live at `sim/L0/rl/tests/custom_task.rs` and `sim/L0/rl/tests/competition.rs`); rewrite their `sim_ml_bridge::` imports to `sim_rl::`. Update workspace member list and dep alias per §9. **No intra-`src/` import rewrites** — those already happened in commit 2 per §1.4. | `cargo build -p sim-rl` green; `cargo test -p sim-rl --test custom_task` green; `cargo xtask grade sim-rl` = A |
| 4 | `refactor: rewrite sim_ml_bridge:: imports across tree` | Apply §8.1–§8.3 tables. Docs too. No crate structure changes. Does **not** touch sim-opt — that lands in commit 5. | `cargo build -p <crate>` per touched crate; `cargo test -p <crate>` where applicable. `cargo xtask grade sim-ml-chassis` and `cargo xtask grade sim-rl` still = A |
| 5 | `refactor(sim-opt): re-home on sim-ml-chassis + sim-rl` | Apply §8.4. Update `sim-opt/Cargo.toml` (regular dep `sim-ml-chassis`, dev-dep `sim-rl`). Rewrite `src/{algorithm,richer_sa,parallel_tempering,analysis}.rs` and `tests/d2c_sr_rematch{,_richer_sa,_pt}.rs` from `sim_ml_bridge::` → `sim_ml_chassis::` / `sim_rl::`. Update `src/lib.rs` module-doc prose. | `cargo build -p sim-opt` green; `cargo test -p sim-opt --release` on non-ignored tests green; the three `#[ignore]`'d rematch fixtures still compile; `cargo xtask grade sim-opt` = A |

**All five commits must land in the same PR.** The chassis/rl split is one
breaking change; partial merges would leave the tree in a half-renamed state
that no downstream branch could rebase cleanly on. Commits 6-8 from earlier
drafts of this spec (scaffold sim-opt, SA `train()` body, D2c rematch test)
shipped separately via the sim-opt path (PRs #187/#188/#190) and are not
part of this PR.

---

## 11. Grading checkpoints

Per-crate A-grade is required. Run each of these before moving on:

```
cargo xtask grade sim-ml-chassis    # after commit 2
cargo xtask grade sim-rl            # after commit 3
cargo xtask grade sim-opt           # after commit 5
```

**If a crate drops from A.** Fix the criterion that dropped before the
next commit — do not batch fixes across commits. The commits above are
structured so each one leaves the tree in a buildable, gradable state.

**Do not run.** `cargo xtask check` or `cargo test` (workspace-level).
Both are explicitly forbidden by CLAUDE.md.

---

## 12. What is explicitly NOT in this PR

Listed so the execution session doesn't wander:

- **Parallel Tempering implementation.** Stub only.
- **CMA-ES implementation.** Stub only.
- **Metropolis-Hastings wrapper.** Stub only.
- **Differential Evolution.** Not even a stub — add to `sim-opt` later.
- **MLP/autograd RL baselines for the rematch.** Linear-only.
- **Bevy visualization for SA or any `sim-opt` algorithm.** Headless.
- **`sim-ml-bridge` backwards-compat shim.** Gone means gone. No
  re-export crate, no `#[deprecated]` aliases.
- **Moving `GibbsSampler` out of `sim-thermostat`.** Stays where it is.
- **Changes to stock reaching tasks** (`reaching_2dof`,
  `reaching_6dof`, `obstacle_reaching_6dof`). They move into
  `sim-ml-chassis` as-is.
- **Refactoring stock tasks to use the new `TaskConfig::from_build_fn`
  constructor.** The private struct-literal path stays; the public
  `from_build_fn` is a cross-crate bypass, not a replacement. See §6.4.
- **Tuning SA hyperparameters past the defaults in §5.5.** That's a
  follow-up if the rematch comes back flat.
- **Writing `build_ppo_mlp`, `build_sac_autograd`, etc.** Linear-only
  builders in this PR.

---

## 13. Memory updates after merge

After the PR lands, update these memory files in a separate commit
(`chore(memory): sync with sim-ml-renovation split`):

- `MEMORY.md` — update the `sim-ml-bridge` line in "Codebase Structure"
  to list `sim-ml-chassis`, `sim-rl`, `sim-opt`.
- `project_sim_ml_bridge.md` → **rename** to `project_sim_ml_split.md`,
  record the split outcome and current grade status of all three crates.
- `project_sim_ml_renovation.md` — note that the three-crate split
  closed the renovation work; keep the doc for context on how we got
  here.
- `project_thermo_rl_loop_vision.md` — add a line noting that
  `sim-opt` is now the home for custom thermo-RL algorithms.

No memory edits during the PR itself. Point-in-time observations get
recorded once the facts stabilise.
