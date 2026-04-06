# ML Competition Framework Spec

> **Status**: Draft
> **Crate**: sim-ml-bridge (extensions) + new competition test harness
> **Branch**: feature/sim-ml-bridge

## Problem

We have three hand-coded algorithms (CEM, REINFORCE, PPO) running on a
2-DOF reaching arm with 10 actor params. The results tell the wrong story:

| Algorithm | Reward improvement | Done triggers | Params |
|-----------|-------------------|---------------|--------|
| CEM       | ~50%              | **>=10/epoch**| 10     |
| REINFORCE | **91%**           | 0-3/epoch     | 10     |
| PPO       | 88%               | 0/epoch       | 15     |

CEM wins on reaches, REINFORCE wins on reward, PPO adds overhead. This
inverts the expected CEM < REINFORCE < PPO ordering because the problem
is too small — 10 params is trivial for evolutionary search and low-
variance enough for raw policy gradients.

## Goal

Build an abstraction layer and scaled-up task where:

1. **5 algorithms** (CEM, REINFORCE, PPO, SAC, TD3) compete on
   standardized tasks with comparable metrics
2. **The ordering is honest** — algorithms win where theory predicts
   they should, and we can form hypotheses and verify them
3. **Adding algorithm N+1 is cheap** — implement one trait, plug in
4. **Adding task M+1 is cheap** — define one config, run all algorithms

## Algorithm landscape

### The five algorithms and their structural differences

| Property | CEM | REINFORCE | PPO | SAC | TD3 |
|----------|-----|-----------|-----|-----|-----|
| Family | Evolutionary | On-policy PG | On-policy PG | Off-policy AC | Off-policy AC |
| Data usage | Whole-rollout fitness | Per-step, use once | Per-step, use K times | Per-step, replay forever | Per-step, replay forever |
| Episode structure | Episodic (reset between generations) | Episodic | Episodic | Continuous (auto-reset) | Continuous (auto-reset) |
| Gradient? | No | Policy gradient | Clipped surrogate | Policy + 2 Q-functions + entropy | Policy + 2 Q-functions |
| Exploration | Parameter perturbation | Gaussian action noise | Gaussian action noise | Entropy-maximizing | Target policy smoothing |
| Key strength | Simple, no gradients | Unbiased gradient | Stable, sample-efficient (on-policy) | Sample-efficient (off-policy), automatic exploration | Stable off-policy, simple |

### The three structural fault lines

**1. What data gets stored:**
- CEM: scalar fitness per rollout
- REINFORCE/PPO: per-step (obs, action, reward, mu_old, v_old) trajectories
- SAC/TD3: per-step (s, a, r, s', done) transitions in a replay buffer

**2. When updates happen:**
- CEM: after all envs complete one episode
- REINFORCE: same
- PPO: same, but K passes over the data
- SAC/TD3: every step (after warmup), using random mini-batches from replay

**3. What gets optimized:**
- CEM: distribution over policy params (mean, std)
- REINFORCE: policy params via gradient ascent
- PPO: policy params + value function params
- SAC: policy + 2 Q-functions + entropy temperature (auto-tuned)
- TD3: policy + 2 Q-functions (delayed actor updates)

### Why this set of five

These five span the three major families (evolutionary, on-policy,
off-policy) with enough overlap to isolate individual design choices:

- CEM vs REINFORCE: gradient-free vs gradient-based (same data)
- REINFORCE vs PPO: raw PG vs clipped surrogate + baseline (same family)
- PPO vs SAC: on-policy vs off-policy (both actor-critic)
- SAC vs TD3: entropy-regularized vs deterministic (same family)

Each pair isolates one conceptual difference.

## Architecture

### Layer 1: Policy and Value Function traits

The policy maps observations to actions. The value function maps
observations to scalar estimates. Both own their parameters and can
compute gradients analytically (no autograd).

```
trait Policy: Send + Sync {
    fn n_params(&self) -> usize;
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);

    /// Forward pass: obs -> mean action
    fn forward(&self, obs: &[f32]) -> Vec<f64>;

    /// d/dtheta log pi(a|s) for Gaussian policy with given sigma.
    /// Returns gradient w.r.t. all params.
    /// Only needed by gradient-based algorithms (not CEM).
    fn log_prob_gradient(
        &self, obs: &[f32], action: &[f64], sigma: f64,
    ) -> Vec<f64>;
}

trait ValueFn: Send + Sync {
    fn n_params(&self) -> usize;
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);

    /// V(s) or Q(s,a) — forward pass
    fn forward(&self, obs: &[f32]) -> f64;

    /// Gradient of MSE loss: d/d_params (forward(obs) - target)^2
    fn mse_gradient(&self, obs: &[f32], target: f64) -> Vec<f64>;
}
```

**Implementations (Phase 1):**

```
LinearPolicy    — tanh(W * s_scaled + b)
                  Current 10-param version for 2-DOF
MlpPolicy       — tanh(W2 * tanh(W1 * s + b1) + b2)
                  Single hidden layer, hand-coded backprop

LinearValue     — w . s_scaled + b
                  Current 5-param version
MlpValue        — w2 . tanh(W1 * s + b1) + b2
                  Single hidden layer
```

For SAC/TD3, the Q-function takes (obs, action) as input:

```
trait QFunction: Send + Sync {
    fn n_params(&self) -> usize;
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);

    /// Q(s, a)
    fn forward(&self, obs: &[f32], action: &[f64]) -> f64;

    /// Gradient of MSE loss w.r.t. params
    fn mse_gradient(
        &self, obs: &[f32], action: &[f64], target: f64,
    ) -> Vec<f64>;

    /// Gradient of Q(s, a) w.r.t. action (for policy improvement)
    fn action_gradient(&self, obs: &[f32], action: &[f64]) -> Vec<f64>;
}
```

**OBS_SCALE normalization**: Policies and value functions accept raw
`&[f32]` observations. Normalization (OBS_SCALE) is configured at
construction time, not baked into the trait. This lets the same policy
type work across tasks with different observation scales.

### Layer 2: Task definition

A task is a reusable environment configuration. It encapsulates
everything needed to construct a VecEnv: MJCF, spaces, reward, done,
truncation, sub-steps, and observation normalization.

```
struct TaskConfig {
    name: &'static str,
    mjcf: &'static str,
    obs_builder: fn(&Model) -> ObservationSpace,
    act_builder: fn(&Model) -> ActionSpace,
    reward_fn: Arc<dyn Fn(&Model, &Data) -> f64 + Send + Sync>,
    done_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    truncated_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    sub_steps: usize,
    obs_scale: Vec<f64>,     // normalization factors per obs dim
    episode_timeout: f64,     // seconds
}

impl TaskConfig {
    fn build_vec_env(&self, n_envs: usize) -> VecEnv;
    fn obs_dim(&self) -> usize;
    fn act_dim(&self) -> usize;
}
```

**Built-in tasks:**
- `TaskConfig::reaching_2dof()` — current 2-joint arm (4 obs, 2 act)
- `TaskConfig::reaching_6dof()` — scaled-up 6-joint arm (12 obs, 6 act)

### Layer 3: Algorithm trait

This is the core abstraction. The trait is deliberately thin — each
algorithm owns its entire training logic. The runner only needs to
know how to start it and collect results.

```
struct EpochMetrics {
    epoch: usize,
    mean_reward: f64,
    done_count: usize,
    total_steps: usize,
    wall_time_ms: u64,
    extra: BTreeMap<String, f64>,  // algorithm-specific
}

struct RunResult {
    algorithm: String,
    task: String,
    epochs: Vec<EpochMetrics>,
    total_wall_time_ms: u64,
}

trait Algorithm {
    fn name(&self) -> &str;

    /// Run the full training loop on the given task.
    /// Returns one EpochMetrics per evaluation point.
    ///
    /// The algorithm owns the VecEnv interaction loop internally.
    /// This is the coarsest possible trait boundary — it avoids
    /// forcing a shared rollout interface across fundamentally
    /// different data requirements.
    fn train(
        &mut self,
        task: &TaskConfig,
        n_envs: usize,
        budget: TrainingBudget,
        seed: u64,
    ) -> Vec<EpochMetrics>;
}

enum TrainingBudget {
    Epochs(usize),       // For episodic algorithms (CEM, REINFORCE, PPO)
    Steps(usize),        // For continuous algorithms (SAC, TD3)
}
```

**Why `train()` is monolithic** (the key design decision):

The five algorithms have fundamentally different inner loops:

- CEM: perturb params → run all envs → rank → select elites
- REINFORCE: sample policy → run all envs → compute returns → gradient step
- PPO: sample policy → run all envs → GAE → K clipped passes
- SAC: sample policy → step → store transition → sample batch → update
  critics → update actor → update temperature → soft-update targets
- TD3: sample policy → step → store transition → sample batch → update
  critics → (every d steps) update actor → update targets

Trying to factor these into shared sub-steps (collect/update) forces
awkward abstractions. The `act()/observe()` step-level trait was
considered and rejected because:

1. CEM doesn't use per-step observations at all
2. SAC/TD3 need the replay buffer to be internal state
3. PPO's K-pass loop doesn't fit step-level granularity
4. The runner would need mode flags (episodic vs continuous) that
   leak algorithm internals

Instead, each algorithm owns its complete loop. The trait boundary
is just "take a task, produce metrics." This is boring — and that's
the point. The complexity lives inside each algorithm where it
belongs, not in shared machinery that tries to be everything.

**What algorithms emit via `extra` metrics:**

| Algorithm | Extra metrics |
|-----------|--------------|
| CEM | elite_mean_fitness, population_std |
| REINFORCE | grad_norm, baseline_variance |
| PPO | value_loss, clip_fraction, actor_grad_norm |
| SAC | q1_loss, q2_loss, policy_loss, entropy, alpha |
| TD3 | q1_loss, q2_loss, policy_loss |

### Layer 4: Competition runner

The competition runner is a test utility, not a runtime framework.
It runs multiple algorithms on the same task and produces comparable
results.

```
struct Competition {
    task: TaskConfig,
    n_envs: usize,
    budget: TrainingBudget,
    seeds: Vec<u64>,
}

struct CompetitionResult {
    task: String,
    runs: Vec<RunResult>,  // one per (algorithm, seed) pair
}

impl Competition {
    fn run(&self, algorithms: &mut [&mut dyn Algorithm]) -> CompetitionResult;
    fn print_summary(result: &CompetitionResult);
}
```

The `print_summary` produces a table like:

```
Task: reaching_6dof (12 obs, 6 act, 50 envs, 60 epochs)
Seed: 42

Algorithm   | Improvement | Done/epoch | Wall time | Key metric
------------|-------------|------------|-----------|------------------
CEM         |   23%       |  0/50      |   4.2s    | pop_std=0.08
REINFORCE   |   71%       |  2/50      |   3.8s    | grad_norm=0.12
PPO         |   89%       | 18/50      |   5.1s    | value_loss=42.3
SAC         |   94%       | 31/50      |   8.7s    | entropy=1.24
TD3         |   91%       | 26/50      |   7.9s    | q_loss=18.7
```

### Layer 5: Shared components

These are internal building blocks used by multiple algorithms, but
NOT part of the trait boundary.

**Adam optimizer** (generic):
```
struct Adam {
    params: Vec<f64>,
    m: Vec<f64>,
    v: Vec<f64>,
    t: usize,
    lr: f64,
    beta1: f64,
    beta2: f64,
    eps: f64,
}
```

Used by: REINFORCE, PPO, SAC, TD3.

**Replay buffer** (for off-policy):
```
struct ReplayBuffer {
    capacity: usize,
    transitions: Vec<Transition>,
    // ring buffer semantics
}

struct Transition {
    obs: Vec<f32>,
    action: Vec<f64>,
    reward: f64,
    next_obs: Vec<f32>,
    done: bool,
}
```

Used by: SAC, TD3.

**GAE computation** (generalized advantage estimation):
```
fn compute_gae(
    rewards: &[f64],
    values: &[f64],
    next_value: f64,
    gamma: f64,
    lambda: f64,
) -> (Vec<f64>, Vec<f64>)  // (advantages, value_targets)
```

Used by: PPO (and potentially A2C if added later).

**Episodic collector** (run all envs to completion):
```
struct EpisodicCollector { ... }

impl EpisodicCollector {
    fn collect(
        env: &mut VecEnv,
        policy: &dyn Policy,
        sigma: f64,
        rng: &mut StdRng,
    ) -> Vec<Trajectory>;
}
```

Used by: CEM (for fitness eval), REINFORCE, PPO. Each algorithm
wraps this with its own trajectory type (CEM only needs rewards,
PPO also needs mu_old/v_old).

## Scaled-up task: 6-DOF reaching arm

### Why 6-DOF

At 614 actor params (MLP with 32-unit hidden layer):
- **CEM** needs population ~2-10x param count = 1200-6000. With 50
  envs, that's 24-120 generations just to evaluate one population.
  CEM becomes sample-starved.
- **REINFORCE** has 614-dimensional gradient to estimate from noisy
  returns. High variance, slow convergence.
- **PPO** with a learned V(s) reduces per-state variance, making the
  614-dim gradient tractable. K passes squeeze more signal per epoch.
- **SAC/TD3** with replay buffer reuse every transition ~100x,
  radically better sample efficiency.

This is the regime where the theoretical ordering holds.

### MJCF

3-segment planar arm (6 hinge joints, alternating axes for 3D reach):

```xml
<mujoco model="reaching-arm-6dof">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <worldbody>
    <body name="seg1" pos="0 0 0">
      <joint name="j1" type="hinge" axis="0 -1 0" damping="2.0"
             limited="true" range="-3.14 3.14"/>
      <joint name="j2" type="hinge" axis="0 0 1" damping="1.5"
             limited="true" range="-1.57 1.57"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.5"/>
      <body name="seg2" pos="0.3 0 0">
        <joint name="j3" type="hinge" axis="0 -1 0" damping="1.5"
               limited="true" range="-2.6 2.6"/>
        <joint name="j4" type="hinge" axis="0 0 1" damping="1.0"
               limited="true" range="-1.57 1.57"/>
        <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.025" mass="0.3"/>
        <body name="seg3" pos="0.25 0 0">
          <joint name="j5" type="hinge" axis="0 -1 0" damping="1.0"
                 limited="true" range="-2.6 2.6"/>
          <joint name="j6" type="hinge" axis="0 0 1" damping="0.5"
                 limited="true" range="-1.57 1.57"/>
          <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.2"/>
          <site name="fingertip" pos="0.2 0 0" size="0.015"/>
        </body>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1" gear="10" ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j2" gear="8"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j3" gear="6"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j4" gear="5"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j5" gear="4"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j6" gear="3"  ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
```

Observation: 12-dim (6 qpos + 6 qvel)
Action: 6-dim (6 motor torques)
Reward: -(qpos - target_qpos)^2 (joint-space, same as 2-DOF)
Done: fingertip within 5cm of target AND low velocity
Truncated: time > 5.0s (longer episodes for 6 joints)

### Policy sizes

| Policy | Params | Formula |
|--------|--------|---------|
| Linear (12→6) | 78 | W[6x12] + b[6] |
| MLP (12→32→6) | 614 | W1[32x12] + b1[32] + W2[6x32] + b2[6] |

| Value fn | Params | Formula |
|----------|--------|---------|
| Linear V(s) | 13 | w[12] + b |
| MLP V(s) | 417 | W1[32x12] + b1[32] + w2[32] + b2 |
| Linear Q(s,a) | 19 | w[18] + b |
| MLP Q(s,a) | 609 | W1[32x18] + b1[32] + w2[32] + b2 |

### Expected ordering on 6-DOF with MLP

| Algorithm | Expected improvement | Expected reaches | Why |
|-----------|---------------------|-----------------|-----|
| CEM | 20-40% | 0 | 614-dim search space, sample-starved |
| REINFORCE | 60-75% | 0-5 | High-variance 614-dim gradient |
| PPO | 85-95% | 10-30 | Learned baseline reduces variance |
| SAC | 90-98% | 25-40 | Off-policy reuse, entropy exploration |
| TD3 | 88-95% | 20-35 | Off-policy, less exploration than SAC |

These are predictions to be verified. The competition framework
exists to test hypotheses like these efficiently.

## MLP backpropagation (hand-coded)

Single hidden layer with tanh activation. No autograd needed — the
chain rule for one layer is straightforward.

### Forward pass

```
h = tanh(W1 * s + b1)        // hidden: [H]
mu = tanh(W2 * h + b2)       // output: [A]
```

### Backward pass (policy gradient)

Given d/d_mu log pi(a|s) (the score function from the Gaussian):

```
score_a = (a - mu) / sigma^2 * (1 - mu^2)     // [A]

// Output layer gradients:
dW2[a,h] = score_a[a] * h[h]                   // [A x H]
db2[a]   = score_a[a]                           // [A]

// Backprop through hidden layer:
d_h[h] = sum_a score_a[a] * W2[a,h]            // [H]
d_z1[h] = d_h[h] * (1 - h[h]^2)               // tanh derivative

dW1[h,o] = d_z1[h] * s[o]                      // [H x O]
db1[h]   = d_z1[h]                             // [H]
```

### Value function backward pass

```
V = w2 . tanh(W1 * s + b1) + b2

dL/dV = 2 * (V - target)

dw2[h] = dL/dV * h[h]
db2    = dL/dV
dW1[h,o] = dL/dV * w2[h] * (1 - h[h]^2) * s[o]
db1[h]   = dL/dV * w2[h] * (1 - h[h]^2)
```

All gradients are O(H * max(O, A)) per sample — fast enough for
H=32 without batched linear algebra.

## Implementation plan

### Phase 1: Abstractions in sim-ml-bridge

1. `Policy` trait + `LinearPolicy` + `MlpPolicy` implementations
2. `ValueFn` trait + `LinearValue` + `MlpValue` implementations
3. `QFunction` trait + `LinearQ` + `MlpQ` implementations
4. `Adam` optimizer (generic, Vec-based)
5. `ReplayBuffer`
6. `GAE` computation (standalone function)
7. `TaskConfig` struct + `reaching_2dof()` + `reaching_6dof()`
8. `EpochMetrics`, `RunResult`, `TrainingBudget`
9. `Algorithm` trait
10. `Competition` runner

Tests: gradient finite-difference checks for all MLP variants,
replay buffer correctness, GAE correctness.

### Phase 2: Algorithm implementations

Each algorithm is a struct implementing `Algorithm`. They live in
sim-ml-bridge (not in examples) so the competition tests can use them.

1. `Cem` — rewrite of current CEM logic using `Policy` trait
2. `Reinforce` — rewrite using `Policy` + `Adam`
3. `Ppo` — rewrite using `Policy` + `ValueFn` + `Adam` + `GAE`
4. `Sac` — new: `Policy` + 2x `QFunction` + `ReplayBuffer` + `Adam`
   + entropy temperature auto-tuning
5. `Td3` — new: `Policy` + 2x `QFunction` + `ReplayBuffer` + `Adam`
   + delayed actor update + target policy smoothing

Tests per algorithm: convergence on 2-DOF (regression), gradient
checks, algorithm-specific invariants.

### Phase 3: Competition tests

```rust
#[test]
fn two_dof_linear_competition() {
    // All 5 algorithms on the easy task with linear policies.
    // CEM should still do well here (10-dim search is trivial).
    // This is the regression test — existing behavior preserved.
}

#[test]
fn six_dof_mlp_competition() {
    // THE test. 5 algorithms on the hard task with MLP policies.
    // Expected ordering: CEM << REINFORCE < PPO < TD3 <= SAC
}

#[test]
fn sample_efficiency_comparison() {
    // Same task, same budget (total env steps), different algorithms.
    // Off-policy (SAC/TD3) should dominate at low sample budgets.
}

#[test]
fn linear_vs_mlp_policy_comparison() {
    // Same algorithm (PPO), same task (6-DOF), linear vs MLP policy.
    // MLP should significantly outperform linear.
}
```

### Phase 4: Visual examples (updated)

The existing Bevy examples (CEM, REINFORCE, PPO) are updated to use
the abstracted `Algorithm` + `TaskConfig`, but remain standalone
single-file examples. New SAC/TD3 examples follow the same pattern.

The visual examples are NOT abstracted into a shared Bevy framework —
they stay as readable, self-contained files per the museum plaque
principle.

## What lives where

```
sim/L0/ml-bridge/src/
    lib.rs              — existing VecEnv, Tensor, spaces
    policy.rs           — Policy trait + LinearPolicy + MlpPolicy
    value.rs            — ValueFn trait + LinearValue + MlpValue
    q_function.rs       — QFunction trait + LinearQ + MlpQ
    adam.rs              — Generic Adam optimizer
    replay_buffer.rs    — ReplayBuffer for off-policy
    gae.rs              — GAE computation
    task.rs             — TaskConfig + built-in tasks
    algorithm.rs        — Algorithm trait + EpochMetrics + Competition
    algorithms/
        cem.rs
        reinforce.rs
        ppo.rs
        sac.rs
        td3.rs

examples/fundamentals/sim-ml/
    vec-env/
        auto-reset/     — CEM visual example (updated)
        reinforce/      — REINFORCE visual example (updated)
        ppo/            — PPO visual example (updated)
        sac/            — SAC visual example (new)
        td3/            — TD3 visual example (new)
```

## Risks and mitigations

**Risk: MLP backprop bugs.** Hand-coded gradients for single-layer
MLPs are straightforward but error-prone at scale. **Mitigation:**
every Policy/ValueFn/QFunction implementation gets a finite-difference
gradient test. These are the first tests written, before any algorithm
code.

**Risk: SAC/TD3 complexity.** Off-policy methods have more moving
parts (replay buffer, target networks, soft updates, entropy tuning).
**Mitigation:** implement TD3 first (simpler — no entropy), then SAC
builds on the same replay buffer + twin Q infrastructure.

**Risk: 6-DOF arm physics.** More joints means more complex dynamics
and potentially harder-to-tune rewards. **Mitigation:** verify the
6-DOF arm works with a hand-tuned controller first (known-good IK
solution), then train algorithms.

**Risk: Over-abstraction.** The trait system could become a framework
that obscures rather than clarifies. **Mitigation:** the Algorithm
trait has ONE method (train). Shared components (Adam, ReplayBuffer,
GAE) are standalone utilities, not mandatory base classes. An
algorithm can ignore all shared components and implement train()
from scratch.

## Hypotheses to test

These are the scientific questions the competition framework is
designed to answer:

1. **CEM scales poorly with param count.** Predict: CEM goes from
   >=10 reaches (2-DOF, 10 params) to 0 reaches (6-DOF, 614 params)
   with the same env budget.

2. **PPO's value function matters at scale.** Predict: PPO >> REINFORCE
   on 6-DOF (>15pp improvement gap), but PPO ~ REINFORCE on 2-DOF
   (<5pp gap).

3. **Off-policy is more sample-efficient.** Predict: SAC reaches 80%
   improvement in 1/3 the env steps that PPO needs.

4. **MLP >> linear for complex tasks.** Predict: MLP policy doubles
   the done trigger count vs linear on 6-DOF, even with the same
   algorithm.

5. **Entropy helps exploration.** Predict: SAC > TD3 on done triggers
   because entropy regularization discovers the precise reaching
   behavior that deterministic TD3 misses.

Each hypothesis becomes a test assertion. If a hypothesis fails,
that's a finding worth documenting — not a bug to fix.
