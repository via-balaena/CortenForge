# Physics-Aware ML Pivot — Construction Spec

This is the mechanical "what" to accompany `physics_aware_ml_pivot.md`'s
strategic "why." Every file move, import rewrite, and grading checkpoint
is spelled out so the execution session makes **no design calls** on the
fly. Scope is the `sim-ml-bridge` → `sim-ml-chassis` + `sim-rl` split;
sim-opt's algorithms and rematch fixtures already shipped via PRs
#187/#188/#190 and are not part of this PR.

Read this document top to bottom. Each numbered section in §10 maps to
one commit.

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

`sim/L0/opt/` already exists and is not created by this PR. See §5.

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
lib.rs
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

These edits happen in commit 2, while the package is still named
`sim-ml-bridge` on disk — commit 3 is a pure rename. Once commit 2
moves `algorithm.rs`, `best_tracker.rs`, etc. out, the remaining
algorithm files no longer compile until their imports are rewritten.

- **`sim-ml-bridge/Cargo.toml`** — add `sim-ml-chassis = { workspace = true }`
  as a regular dep (one-commit lifetime; dropped in commit 3 when the
  new `sim-rl` Cargo.toml replaces it per §4.1).
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

**Why this relocation is load-bearing.** The visual SAC example
(`vec-env/sac/src/main.rs:45`) builds SAC from chassis primitives — no
`Sac` struct — so it's a chassis-only consumer. Leaving
`gaussian_log_prob` in `sim-rl::sac` would force the example to pull
`sim-rl` in for a single math helper. Moving it to
`sim-ml-chassis::stats` keeps the example on a single-crate dependency.
`sim-rl`'s `sac.rs` also calls this function internally at lines 397
and 519; those imports redirect to chassis in the same commit.

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

---

## 5. `sim-opt` — already shipped

sim-opt shipped via PRs #187/#188/#190 (2026-04-12 through 2026-04-15).
The crate exists at `sim/L0/opt/` with four source modules and three
rematch fixtures:

- `src/algorithm.rs::Sa` — basic Simulated Annealing implementing
  `Algorithm` (PR #188, commit `3d6f9ad8`).
- `src/richer_sa.rs::RicherSa` — SA with Rechenberg 1/5 success rule
  (PR #190, commit `c7aabcc7` + self-review fix `abf0f3aa`).
- `src/parallel_tempering.rs::Pt` — K=4 geometric-ladder Parallel
  Tempering, swap-every-epoch (PR #190, commit `8f8e006f`).
- `src/analysis.rs` — dual-metric `TwoMetricOutcome` rematch analysis
  pipeline with bootstrap CIs and the Ch 30 three-outcome classifier
  (PR #190, commit `086c04c8`).
- `tests/d2c_sr_rematch{,_richer_sa,_pt}.rs` — three `#[ignore]`'d
  integration fixtures, each ~4h on an MBP under `--release`.

28 unit tests green under `cargo test -p sim-opt --release`. The D2c-SR
rematch is closed at the study level per Ch 53 §6.4.

**The chassis/rl split does not touch sim-opt's algorithms or tests** — it
only re-homes the crate's `sim_ml_bridge::` imports onto
`sim_ml_chassis::` and `sim_rl::`. That rewrite is commit 5 of §10. The
file-by-file rewrite table is §8.4.

sim-opt's `Cargo.toml`, `src/lib.rs` module doc, and the `use` lines in
the four source files + three test fixtures are the only diff surface in
commit 5. The crate keeps its A grade across the re-home.

---

## 6. SR task packaging — deferred

The construction spec originally proposed extracting `stochastic_resonance()`
into `sim_thermostat::tasks`. Ch 42 §6 sub-decision (f) intentionally kept the
SR MJCF + constants duplicated in the three rematch fixtures under
`sim/L0/opt/tests/`. The chassis/rl split does not revisit that decision. If a
future PR wants SR-as-`TaskConfig`, see `project_thermo_rl_loop_vision.md` for
the motivation.

---

## 7. D2c rematch — shipped

Shipped via `sim/L0/opt/tests/d2c_sr_rematch{,_richer_sa,_pt}.rs` under
PRs #187/#188/#190. The rematch ran three times (basic SA, richer-SA with
Rechenberg 1/5, Parallel Tempering K=4) on the same MacBook Pro; Ch 53 §6.4
of the ml-chassis-refactor study closes the D2c-SR question at the study
level. See `physics_aware_ml_pivot.md` "D2c rematch — shipped and closed"
and `docs/studies/ml_chassis_refactor/src/53-robustness-check-prereg.md`
§§5-6 for the honest-reading asymmetry (richer-SA weak corroborator, PT
carries the differential signal).

The chassis/rl split does not re-derive any of this; it only rewrites
sim-opt's `sim_ml_bridge::` imports per §8.4.

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
| 3 | `refactor(sim-rl): rename sim-ml-bridge and re-export chassis types` | Rename `sim/L0/ml-bridge/` → `sim/L0/rl/`. Replace the Cargo.toml contents per §4.1 (drops the transitional `sim-ml-chassis` dep line from commit 2, adds `approx` + `sim-mjcf` dev-deps, changes package name to `sim-rl`). Rewrite `src/lib.rs` per §4.2 (re-exports include `TrainingProvenance`, `PolicyArtifact`, `TrainingCheckpoint`). The two integration tests travel with the rename (they live at `sim/L0/rl/tests/custom_task.rs` and `sim/L0/rl/tests/competition.rs`); rewrite their `sim_ml_bridge::` imports to `sim_rl::`. Update workspace member list and dep alias per §9. **No intra-`src/` import rewrites** — those already happened in commit 2 per §1.4. | `cargo build -p sim-rl` green; `cargo test -p sim-rl --test custom_task` green; `cargo xtask grade sim-rl` = A |
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

- **New algorithms in sim-opt.** CMA-ES, Metropolis-Hastings, Differential
  Evolution — not in this PR. SA, richer-SA, and PT already shipped; this
  PR only re-homes their imports.
- **MLP/autograd RL baselines for the rematch.** Linear-only. The MLP
  rematch remains a deferred follow-up per `project_sim_ml_pivot.md`.
- **Bevy visualization for sim-opt algorithms.** Headless.
- **`sim-ml-bridge` backwards-compat shim.** Gone means gone. No
  re-export crate, no `#[deprecated]` aliases.
- **Moving `GibbsSampler` out of `sim-thermostat`.** Stays where it is.
- **Changes to stock reaching tasks** (`reaching_2dof`,
  `reaching_6dof`, `obstacle_reaching_6dof`). They move into
  `sim-ml-chassis` as-is.
- **Refactoring stock tasks to use `TaskConfig::from_build_fn`.** The
  private struct-literal path stays; the public `from_build_fn` (shipped
  in PR #190) is a cross-crate bypass, not a replacement.
- **Tuning SA / richer-SA / PT hyperparameters.** Frozen as of
  Chs 50-55 of the ml-chassis-refactor study.
- **New builder factories for `Competition`.** Every fixture in the
  tree constructs algorithms with direct `::new` calls; no `build_*`
  functions exist or are needed.

---

## 13. Memory updates after merge

After the PR lands, update these memory files in a separate commit
(`chore(memory): sync with chassis/rl split`):

- `MEMORY.md` — update the `sim/L0/ml-bridge/` line in "Codebase
  Structure" to list `sim-ml-chassis` + `sim-rl` (sim-opt is already
  listed).
- `project_sim_ml_pivot.md` — flip the "chassis/rl split is pending"
  paragraph to "shipped", record the PR number, note any surprises
  from commits 1-5.
- `project_sim_ml_bridge.md` → **rename** to `project_sim_ml_split.md`,
  record the final three-crate grade status.
- `project_sim_ml_renovation.md` — close out with a pointer to the
  split PR; keep the doc for historical context.

No memory edits during the PR itself. Point-in-time observations get
recorded once the facts stabilise. The checklist at
`construction_spec_review_checklist.md` also gets deleted in the same
`chore(memory)` commit.
