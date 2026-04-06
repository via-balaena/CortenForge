# ML Competition Framework Spec

> **Status**: Draft
> **Crate**: sim-ml-bridge (extensions)
> **Branch**: feature/sim-ml-bridge

## Vision

CortenForge's ML layer is designed to scale from pedagogical hand-coded
algorithms all the way to differentiable co-design — jointly optimizing
a mechanism's morphology and control policy, then fabricating the result.

This spec defines the architecture at every level of that journey. The
key constraint: each level must be a clean boundary where you can swap
implementations without rewriting the layers above. Hand-coded gradients
today, burn autograd tomorrow, differentiable physics next year — the
Algorithm implementations and competition tests never change.

## The scaling ladder

| Level | What | Param scale | Compute | Unlocks |
|-------|------|------------|---------|---------|
| **0** | Linear policies, hand-coded math | 10-80 | CPU, manual | Pedagogical examples, algorithm comparison |
| **1** | 1-layer MLP, hand-coded backprop | 600 | CPU, manual | Scaled tasks where PPO > REINFORCE > CEM |
| **2** | Deep networks, autograd | 10K-1M | CPU/GPU, burn/candle | Research-grade RL, complex tasks |
| **3** | GPU-accelerated environments | Any | GPU envs + GPU networks | Isaac Gym-style massively parallel training |
| **4** | Differentiable physics | Any | Backprop through sim | Model-based RL with exact dynamics gradients, trajectory optimization |
| **5** | Co-design | Any | End-to-end differentiable | Jointly optimize morphology (cf-design) + controller (policy), then fabricate |

Each level builds on the one below. The architecture is designed so
that reaching level N never requires rewriting level N-1.

### Level 0-1: Hand-coded (this implementation cycle)

What we build first. 5 algorithms, 2 tasks (2-DOF and 6-DOF reaching
arms), linear and single-layer MLP policies with hand-coded gradients.
The competition framework validates that algorithm ordering matches
theory. All infrastructure (traits, tasks, competition runner) is
established here and carries forward to all subsequent levels.

### Level 2: Autograd

Swap the hand-coded `MlpPolicy` for a `BurnPolicy` backed by burn
(or candle). The `Algorithm` implementations don't change — they call
the same `DifferentiablePolicy` trait methods. What changes:

- Networks can be arbitrarily deep (3+ layers, residual connections)
- Gradient computation is automatic (no hand-coded backprop)
- GPU tensor operations available via burn's backends
- New policy architectures (conv for vision, recurrent for POMDP)
  slot in without touching algorithm code

The trait boundary is the firewall: algorithms depend on trait
interfaces, never on concrete policy implementations.

### Level 3: GPU-accelerated environments

CortenForge already has a GPU physics pipeline spec
(`sim/docs/GPU_PHYSICS_PIPELINE_SPEC.md`). When the sim runs on GPU,
the ML layer can run thousands of parallel environments without
CPU-GPU transfer overhead:

- VecEnv backed by GPU BatchSim (observations stay on device)
- Policy forward pass on GPU (burn backend)
- No CPU roundtrip per step — entire rollout is GPU-resident
- Isaac Gym demonstrated 10,000+ envs at real-time — same principle

The `VecEnv` trait and `TaskConfig` stay the same. The implementation
swaps CPU BatchSim for GPU BatchSim. Algorithms see the same
`step() -> VecStepResult` interface.

### Level 4: Differentiable physics

This is where CortenForge diverges from MuJoCo. MuJoCo MPC uses
finite differences to approximate dynamics gradients. A differentiable
sim provides exact analytical gradients:

```
d(state_{t+1}) / d(action_t)     — how actions affect next state
d(state_{t+1}) / d(state_t)      — how state propagates
d(reward_t) / d(action_t)         — direct reward sensitivity
```

This enables:
- **Model-based RL** (PETS, Dreamer) with exact dynamics model
- **Direct trajectory optimization** — backprop through time, no RL
  needed. Compute d(total_reward) / d(action_sequence) analytically.
- **System identification** — fit simulator parameters to real-world
  data via gradient descent
- **Analytical policy gradients** — instead of estimating the policy
  gradient via sampling (REINFORCE), compute it exactly through the
  differentiable dynamics

The sim already computes Jacobians for constraints and contact. The
path to full differentiability is extending this to the complete
forward dynamics pipeline. This is a major project but the architecture
must not prevent it.

**What differentiable physics requires from the ML layer:**
- Tensor type that supports autodiff (burn tensors with gradient
  tracking, or a custom tape)
- Reward functions that are differentiable (not opaque closures)
- A `DifferentiableEnv` trait extending `Environment` with:
  ```
  fn step_differentiable(&self, action: &Tensor) -> DiffStepResult
  // where DiffStepResult carries gradient information
  ```

### Level 5: Co-design

The final integration target. cf-design generates body geometry (SDF-based, implicit
surfaces). sim runs the physics. ML optimizes the control policy.
Co-design closes the loop:

```
morphology_params ──→ cf-design ──→ body geometry
                                        │
                                        ▼
policy_params ──→ policy ──→ actions ──→ sim ──→ reward
    ▲                                              │
    └──────────── gradient ────────────────────────┘
                     │
morphology_params ◄──┘  (level 4: backprop through sim + design)
```

With differentiable physics (level 4) and differentiable geometry
(cf-design's SDF representation is inherently differentiable), the
entire pipeline from morphology parameters to reward is differentiable.
Gradient descent can jointly optimize both.

This integrated pipeline (differentiable geometry + differentiable
physics + policy optimization) is not available in existing toolchains.
MuJoCo is a black-box sim. Isaac Gym does not include a parametric
design system.

**What co-design requires from the ML layer:**
- `MorphologySpace` — parameterizes body geometry (joint lengths,
  link shapes, actuator placements)
- `CoDesignAlgorithm` trait — optimizes both morphology and policy
- Integration with cf-design's SDF representation
- The ability to reconstruct MJCF (or equivalent) from morphology
  params at each optimization step

## Trait architecture

The traits are the stable interfaces that survive all 6 levels. They
are designed with knowledge of the full scaling ladder — no level
should require breaking a trait boundary.

### Compute backend (the foundation)

The traits operate on slices (`&[f32]`, `&[f64]`, `&mut [f64]`), not
framework-specific tensor types. This is deliberate:

- Level 0-1: data lives in plain arrays
- Level 2+: autograd backends convert slices to/from their tensor
  format internally. The trait boundary stays at slices.
- Level 3+: GPU backends materialize slices lazily or batch the
  conversions

If a future level needs zero-copy GPU tensors in the trait signatures,
the traits can be extended with optional methods (default impls that
delegate to the slice versions). The slice-based methods remain as
the universal fallback.

### Policy trait (split for flexibility)

```rust
/// Base policy — enough for evolutionary methods (CEM, CMA-ES).
/// Does not require gradient computation.
trait Policy: Send + Sync {
    fn n_params(&self) -> usize;
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);

    /// Deterministic forward pass: obs -> mean action.
    fn forward(&self, obs: &[f32]) -> Vec<f64>;
}

/// Extends Policy with gradient computation.
/// Required by all gradient-based algorithms.
///
/// At levels 0-1, gradients are hand-coded.
/// At level 2+, an autograd backend computes them automatically
/// from the forward pass — the algorithm never knows the difference.
trait DifferentiablePolicy: Policy {
    /// d/dtheta log pi(a|s) for Gaussian policy with given sigma.
    fn log_prob_gradient(
        &self,
        obs: &[f32],
        action: &[f64],
        sigma: f64,
    ) -> Vec<f64>;
}

/// Extends Policy with a learnable log_std for entropy-based methods.
/// Required by SAC (entropy-regularized exploration).
///
/// Algorithms that use fixed sigma schedules (REINFORCE, PPO) use
/// DifferentiablePolicy. SAC needs learned exploration, so it uses
/// this trait which adds log_std as part of the policy output.
trait StochasticPolicy: DifferentiablePolicy {
    /// Forward pass that also returns log_std per action dimension.
    fn forward_stochastic(&self, obs: &[f32]) -> (Vec<f64>, Vec<f64>);

    /// Gradient of log pi(a|s) w.r.t. all params (including log_std).
    fn log_prob_gradient_stochastic(
        &self,
        obs: &[f32],
        action: &[f64],
    ) -> Vec<f64>;

    /// Entropy of the policy at a given state.
    fn entropy(&self, obs: &[f32]) -> f64;
}
```

**Why the three-level split:**
- CEM only needs `Policy` (forward + param perturbation)
- REINFORCE, PPO, TD3 need `DifferentiablePolicy` (+ gradients)
- SAC needs `StochasticPolicy` (+ learned exploration)

An autograd backend implements all three — `log_prob_gradient` is
derived automatically from the forward pass. Hand-coded backends
implement each manually.

### Value function traits

```rust
/// State value function V(s).
/// Used by PPO (advantage estimation) and optionally A2C.
trait ValueFn: Send + Sync {
    fn n_params(&self) -> usize;
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);

    fn forward(&self, obs: &[f32]) -> f64;
    fn mse_gradient(&self, obs: &[f32], target: f64) -> Vec<f64>;
}

/// State-action value function Q(s, a).
/// Used by SAC and TD3 (twin Q-networks).
trait QFunction: Send + Sync {
    fn n_params(&self) -> usize;
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);

    fn forward(&self, obs: &[f32], action: &[f64]) -> f64;

    /// Gradient of MSE loss w.r.t. Q-function params.
    fn mse_gradient(
        &self,
        obs: &[f32],
        action: &[f64],
        target: f64,
    ) -> Vec<f64>;

    /// Gradient of Q(s, a) w.r.t. action.
    /// Used by TD3 for deterministic policy gradient:
    ///   dJ/dtheta = dQ/da * da/dtheta
    /// Used by SAC for reparameterized policy gradient.
    fn action_gradient(
        &self,
        obs: &[f32],
        action: &[f64],
    ) -> Vec<f64>;
}
```

### Target networks (for off-policy stability)

SAC and TD3 use "target" copies of their Q-networks that are slowly
updated via Polyak averaging (soft update). This is a pattern, not a
trait:

```rust
/// Soft-update target params: target = tau * source + (1-tau) * target
fn soft_update(target: &mut dyn QFunction, source: &dyn QFunction, tau: f64) {
    let src = source.params();
    let tgt = target.params().to_vec();
    let updated: Vec<f64> = tgt.iter().zip(src).map(|(&t, &s)| tau * s + (1.0 - tau) * t).collect();
    target.set_params(&updated);
}
```

This works at any level — it only touches `params()` and `set_params()`.

### Task definition

```rust
struct TaskConfig {
    name: &'static str,
    mjcf: &'static str,
    build_obs: fn(&Model) -> ObservationSpace,
    build_act: fn(&Model) -> ActionSpace,
    reward_fn: Arc<dyn Fn(&Model, &Data) -> f64 + Send + Sync>,
    done_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    truncated_fn: Arc<dyn Fn(&Model, &Data) -> bool + Send + Sync>,
    sub_steps: usize,
    obs_scale: Vec<f64>,
    episode_timeout: f64,
}

impl TaskConfig {
    fn build_vec_env(&self, n_envs: usize) -> VecEnv;
    fn obs_dim(&self, model: &Model) -> usize;
    fn act_dim(&self, model: &Model) -> usize;

    // Built-in tasks
    fn reaching_2dof() -> Self;
    fn reaching_6dof() -> Self;
}
```

At level 3 (GPU envs), `TaskConfig` gains a `build_gpu_vec_env()` that
returns a GPU-backed VecEnv with the same step/reset interface.

At level 4 (differentiable physics), `TaskConfig` gains a
`build_diff_env()` that returns a differentiable environment. The
reward_fn would need to be differentiable (not an opaque closure) —
this is a level-4 concern that doesn't affect levels 0-2.

### Algorithm trait

```rust
struct EpochMetrics {
    epoch: usize,
    mean_reward: f64,
    done_count: usize,
    total_steps: usize,
    wall_time_ms: u64,
    extra: BTreeMap<String, f64>,
}

enum TrainingBudget {
    Epochs(usize),
    Steps(usize),
}

/// The core abstraction. Each algorithm owns its entire training loop.
///
/// This trait has ONE method. The algorithm interacts with VecEnv
/// internally, manages its own data structures (trajectories, replay
/// buffers, populations), and reports standardized metrics.
///
/// The monolithic train() is deliberate — see "Why train() is
/// monolithic" below.
trait Algorithm: Send {
    fn name(&self) -> &str;

    fn train(
        &mut self,
        task: &TaskConfig,
        n_envs: usize,
        budget: TrainingBudget,
        seed: u64,
    ) -> Vec<EpochMetrics>;
}
```

**Why `train()` is monolithic:**

The five core algorithms have fundamentally different inner loops:

| Algorithm | Inner loop |
|-----------|-----------|
| CEM | perturb params → run all envs → rank → select elites |
| REINFORCE | sample policy → run all envs → compute returns → gradient step |
| PPO | sample policy → run all envs → GAE → K clipped passes |
| SAC | step env → store transition → sample batch → update 5 networks → adjust temperature |
| TD3 | step env → store transition → sample batch → update critics → (every d steps) update actor + targets |

Future algorithms are even more divergent:

| Algorithm | Loop structure |
|-----------|---------------|
| CMA-ES | Like CEM but with covariance matrix adaptation |
| A3C | Parallel workers with async gradient updates |
| PETS | Collect data → fit dynamics model ensemble → plan via CEM through model |
| Dreamer | Collect data → fit world model → imagine rollouts → update policy in imagination |
| MAP-Elites | Mutation → evaluate → place in behavior-performance archive |

A step-level trait (`act()/observe()`) was considered and rejected:

1. CEM doesn't use per-step observations — it perturbs params, not actions
2. SAC's replay buffer is internal state that doesn't fit step-level callbacks
3. PPO's K-pass inner loop doesn't map to step-level granularity
4. Model-based methods (PETS, Dreamer) have a completely different
   loop structure (train model → plan through model → collect data)
5. MAP-Elites doesn't even have a sequential training loop

The monolithic `train()` means: each algorithm is a self-contained
implementation. The competition runner doesn't know or care how they
work. Adding algorithm N+1 means implementing one method.

### Competition runner

```rust
struct Competition {
    task: TaskConfig,
    n_envs: usize,
    budget: TrainingBudget,
    seeds: Vec<u64>,
}

struct CompetitionResult {
    task: String,
    runs: Vec<RunResult>,
}

struct RunResult {
    algorithm: String,
    seed: u64,
    epochs: Vec<EpochMetrics>,
    total_wall_time_ms: u64,
}

impl Competition {
    fn run(&self, algorithms: &mut [Box<dyn Algorithm>]) -> CompetitionResult;
    fn print_summary(result: &CompetitionResult);
    fn assert_ordering(result: &CompetitionResult, metric: &str, order: &[&str]);
}
```

The `assert_ordering` method is the hypothesis tester:
```rust
competition.assert_ordering(&result, "mean_reward", &["CEM", "REINFORCE", "PPO", "TD3", "SAC"]);
// Asserts: CEM < REINFORCE < PPO < TD3 < SAC on mean_reward
```

## Algorithm landscape

### The five core algorithms

| Property | CEM | REINFORCE | PPO | SAC | TD3 |
|----------|-----|-----------|-----|-----|-----|
| Family | Evolutionary | On-policy PG | On-policy PG | Off-policy AC | Off-policy AC |
| Uses Policy | `Policy` | `DifferentiablePolicy` | `DifferentiablePolicy` | `StochasticPolicy` | `DifferentiablePolicy` |
| Uses ValueFn | No | No | `ValueFn` (V) | No | No |
| Uses QFunction | No | No | No | 2x `QFunction` | 2x `QFunction` |
| Uses ReplayBuffer | No | No | No | Yes | Yes |
| Target networks | No | No | No | 2x Q targets | 2x Q targets + policy target |
| Data usage | Whole-rollout fitness | Per-step, use once | Per-step, K passes | Per-step, replay forever | Per-step, replay forever |
| Exploration | Param perturbation | Gaussian noise (fixed sigma) | Gaussian noise (fixed sigma) | Entropy maximization (learned) | Target policy smoothing |
| Update frequency | Per generation | Per epoch | Per epoch (K passes) | Per step | Per step |

### Why this set of five spans the design space

Each adjacent pair isolates one conceptual difference:

- **CEM → REINFORCE**: gradient-free → gradient-based
- **REINFORCE → PPO**: raw PG → clipped surrogate + learned baseline
- **PPO → SAC**: on-policy → off-policy, fixed sigma → learned entropy
- **SAC → TD3**: entropy-regularized → deterministic, maximum entropy → target smoothing

### Future algorithms the architecture supports

The trait architecture is designed to accommodate these without changes:

| Algorithm | Family | What it needs | Level |
|-----------|--------|--------------|-------|
| CMA-ES | Evolutionary | `Policy` only | 0 |
| A2C | On-policy | `DifferentiablePolicy` + `ValueFn` | 0-1 |
| TRPO | On-policy | `DifferentiablePolicy` + `ValueFn` + Fisher vector product | 2 |
| DDPG | Off-policy | `DifferentiablePolicy` + `QFunction` | 1 |
| HER | Off-policy + goal relabeling | Any off-policy + goal-conditioned task | 1-2 |
| PETS | Model-based | Dynamics model ensemble + CEM planner | 2 |
| Dreamer | Model-based | World model (RSSM) + imagination rollouts | 2 |
| MAP-Elites | Quality-diversity | `Policy` + behavior descriptor | 0-1 |
| PPG | On-policy | `DifferentiablePolicy` + `ValueFn` + auxiliary heads | 2 |
| DreamerV3 | Model-based | Symlog predictions, discrete world model | 2 |

None of these require changing the `Algorithm` trait. They implement
`train()` with their own internal structure.

TRPO requires a Fisher vector product (second-order optimization).
At level 0-1, this is hand-coded. At level 2+, autograd computes it
via Hessian-vector products. The `DifferentiablePolicy` trait doesn't
need to expose second-order methods — TRPO computes them internally
using the first-order `log_prob_gradient` + finite differences, or
at level 2+ via the autograd backend directly.

## Shared components

### Adam optimizer (generic)

```rust
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

impl Adam {
    fn new(n_params: usize, lr: f64) -> Self;
    fn step(&mut self, gradient: &[f64], ascent: bool);
    fn params(&self) -> &[f64];
    fn set_params(&mut self, params: &[f64]);
}
```

Used by: REINFORCE, PPO, SAC, TD3 (and all gradient-based methods).

At level 2+, this may be replaced by the autograd framework's built-in
optimizer. The `Algorithm` trait doesn't care — Adam is internal.

### Replay buffer

```rust
struct ReplayBuffer {
    capacity: usize,
    obs_dim: usize,
    act_dim: usize,
    // Ring buffer storage
    obs: Vec<f32>,
    actions: Vec<f64>,
    rewards: Vec<f64>,
    next_obs: Vec<f32>,
    dones: Vec<bool>,
    len: usize,
    pos: usize,
}

struct TransitionBatch {
    obs: Vec<f32>,       // [batch_size x obs_dim]
    actions: Vec<f64>,   // [batch_size x act_dim]
    rewards: Vec<f64>,   // [batch_size]
    next_obs: Vec<f32>,  // [batch_size x obs_dim]
    dones: Vec<bool>,    // [batch_size]
}

impl ReplayBuffer {
    fn new(capacity: usize, obs_dim: usize, act_dim: usize) -> Self;
    fn push(&mut self, obs: &[f32], action: &[f64], reward: f64, next_obs: &[f32], done: bool);
    fn sample(&self, batch_size: usize, rng: &mut impl Rng) -> TransitionBatch;
    fn len(&self) -> usize;
}
```

Used by: SAC, TD3 (and all off-policy methods).

At level 3 (GPU), the replay buffer could be GPU-resident.

### GAE computation

```rust
fn compute_gae(
    rewards: &[f64],
    values: &[f64],
    next_value: f64,  // bootstrap: 0 if done, V(s') if truncated
    gamma: f64,
    lambda: f64,
) -> (Vec<f64>, Vec<f64>)  // (advantages, value_targets)
```

Used by: PPO (and A2C, PPG, any GAE-based method).

### Episodic collector

```rust
struct EpisodicRollout {
    obs: Vec<Vec<f32>>,        // [n_envs][steps][obs_dim]
    actions: Vec<Vec<f64>>,    // [n_envs][steps][act_dim]
    rewards: Vec<Vec<f64>>,    // [n_envs][steps]
    dones: Vec<bool>,          // [n_envs] — how each env ended
    terminal_obs: Vec<Option<Vec<f32>>>,  // [n_envs]
}

fn collect_episodic_rollout(
    env: &mut VecEnv,
    act_fn: &mut dyn FnMut(usize, &[f32]) -> Vec<f64>,
    max_steps: usize,
) -> EpisodicRollout;
```

Used by: CEM, REINFORCE, PPO (any episodic method).

### Policy implementations

**Level 0:**
```rust
struct LinearPolicy { ... }      // tanh(W * s_scaled + b)
struct LinearValue { ... }       // w . s_scaled + b
struct LinearQ { ... }           // w . [s_scaled; a] + b
```

**Level 1:**
```rust
struct MlpPolicy { ... }        // tanh(W2 * tanh(W1 * s + b1) + b2)
struct MlpValue { ... }         // w2 . tanh(W1 * s + b1) + b2
struct MlpQ { ... }             // w2 . tanh(W1 * [s; a] + b1) + b2
```

All implement hand-coded `forward()` and gradient methods.

**Level 2+ (future, not implemented now):**
```rust
struct BurnPolicy<B: Backend> { ... }   // arbitrary burn Module
struct CandlePolicy { ... }             // candle-based
```

These implement the same traits via autograd. The algorithm doesn't
know the difference.

## MLP backpropagation (hand-coded, level 1)

Single hidden layer with tanh activation.

### Policy forward + gradient

```
Forward:
    z1 = W1 * s + b1           // [H]
    h  = tanh(z1)              // [H]
    z2 = W2 * h + b2           // [A]
    mu = tanh(z2)              // [A]

Gradient of log pi(a|s) w.r.t. all params:
    score[a] = (a - mu[a]) / sigma^2 * (1 - mu[a]^2)

    dW2[a,h] = score[a] * h[h]
    db2[a]   = score[a]
    d_h[h]   = sum_a( score[a] * W2[a,h] )
    d_z1[h]  = d_h[h] * (1 - h[h]^2)
    dW1[h,o] = d_z1[h] * s[o]
    db1[h]   = d_z1[h]
```

### Value function forward + gradient

```
Forward:
    z1 = W1 * s + b1
    h  = tanh(z1)
    V  = w2 . h + b2

Gradient of (V - target)^2:
    dL_dV = 2 * (V - target)

    dw2[h] = dL_dV * h[h]
    db2    = dL_dV
    d_z1[h]  = dL_dV * w2[h] * (1 - h[h]^2)
    dW1[h,o] = d_z1[h] * s[o]
    db1[h]   = d_z1[h]
```

### Q-function forward + gradient

Same as value function but input is `[s; a]` (concatenated obs + action).
The `action_gradient` (dQ/da) follows the same chain rule through W1's
action columns.

All gradients are O(H * max(O, A)) per sample.

## Scaled-up task: 6-DOF reaching arm

### Why 6-DOF

At 614 MLP actor params:
- **CEM** needs population ~2-10x params = 1200-6000. With 50 envs,
  that's 24-120 generations per population. Sample-starved.
- **REINFORCE** has a 614-dim gradient estimated from noisy returns.
  High variance, slow convergence.
- **PPO** reduces variance via learned V(s), making 614-dim gradients
  tractable. K passes extract more signal per epoch.
- **SAC/TD3** reuse every transition ~100x from replay buffer.
  Significantly better sample efficiency.

### MJCF

3-segment planar arm with 6 hinge joints:

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

- Observation: 12-dim (6 qpos + 6 qvel)
- Action: 6-dim (6 motor torques)
- Reward: -(qpos - target_qpos)^2 (joint-space)
- Done: fingertip within 5cm AND velocity < threshold
- Truncated: time > 5.0s

### Policy and critic sizes

| Network | Params | Architecture |
|---------|--------|-------------|
| Linear actor | 78 | W[6x12] + b[6] |
| MLP actor | 614 | 12 → 32 → 6 |
| Linear V(s) | 13 | w[12] + b |
| MLP V(s) | 417 | 12 → 32 → 1 |
| Linear Q(s,a) | 19 | w[18] + b |
| MLP Q(s,a) | 609 | 18 → 32 → 1 |

### Expected ordering on 6-DOF MLP

| Algorithm | Improvement | Reaches | Why |
|-----------|------------|---------|-----|
| CEM | 20-40% | 0 | 614-dim search, sample-starved |
| REINFORCE | 60-75% | 0-5 | High-variance 614-dim gradient |
| PPO | 85-95% | 10-30 | Learned baseline reduces variance |
| TD3 | 88-95% | 20-35 | Off-policy reuse, less exploration |
| SAC | 90-98% | 25-40 | Off-policy + entropy exploration |

## Implementation plan

### Phase 1: Core abstractions (sim-ml-bridge)

1. `Policy` + `DifferentiablePolicy` + `StochasticPolicy` traits
2. `ValueFn` trait
3. `QFunction` trait
4. `LinearPolicy`, `LinearValue`, `LinearQ` implementations
5. `MlpPolicy`, `MlpValue`, `MlpQ` implementations
6. `Adam` optimizer (Vec-based, generic)
7. `ReplayBuffer`
8. `compute_gae()` standalone function
9. `collect_episodic_rollout()` utility
10. `TaskConfig` + `reaching_2dof()` + `reaching_6dof()`
11. `EpochMetrics`, `TrainingBudget`, `RunResult`
12. `Algorithm` trait
13. `Competition` runner

Gradient finite-difference tests for every forward+gradient pair.

### Phase 2: Algorithm implementations

In sim-ml-bridge, not in examples:

1. `Cem` — `Policy` only, population-based
2. `Reinforce` — `DifferentiablePolicy` + `Adam`
3. `Ppo` — `DifferentiablePolicy` + `ValueFn` + `Adam` + GAE
4. `Td3` — `DifferentiablePolicy` + 2x `QFunction` + `ReplayBuffer`
   + `Adam` + delayed actor + target networks
5. `Sac` — `StochasticPolicy` + 2x `QFunction` + `ReplayBuffer`
   + `Adam` + auto-tuned entropy temperature

TD3 before SAC — TD3 is simpler, shares the twin-Q + replay
infrastructure that SAC builds on.

### Phase 3: Competition tests

```rust
#[test]
fn two_dof_linear_regression() {
    // All 5 on easy task. Verify existing behavior preserved.
}

#[test]
fn six_dof_mlp_ordering() {
    // Primary ordering test. CEM << REINFORCE < PPO < TD3 <= SAC
}

#[test]
fn sample_efficiency() {
    // Same total env steps, different algorithms.
    // Off-policy should dominate at low budgets.
}

#[test]
fn linear_vs_mlp() {
    // Same algorithm, same task, different policy complexity.
    // MLP >> linear on 6-DOF.
}
```

### Phase 4: Visual examples

Update existing CEM/REINFORCE/PPO Bevy examples to use TaskConfig +
Algorithm abstractions. Add SAC and TD3 visual examples. Each remains
a standalone single-file example (museum plaque principle).

### Phase 5+ (future levels)

- Level 2: Add burn backend, implement `BurnPolicy` etc.
- Level 3: GPU VecEnv backed by GPU BatchSim
- Level 4: Differentiable forward dynamics
- Level 5: cf-design integration for co-design

## Hypotheses

Scientific questions the framework is designed to answer. Each becomes
a test assertion.

### Level 0-1 hypotheses

1. **CEM scales poorly with param count.** CEM goes from >=10 reaches
   (2-DOF, 10 params) to 0 reaches (6-DOF, 614 params).

2. **PPO's value function matters at scale.** PPO >> REINFORCE on
   6-DOF (>15pp gap) but PPO ~ REINFORCE on 2-DOF (<5pp gap).

3. **Off-policy is more sample-efficient.** SAC reaches 80% improvement
   in 1/3 the env steps PPO needs.

4. **MLP >> linear for complex tasks.** MLP doubles done triggers vs
   linear on 6-DOF.

5. **Entropy helps exploration.** SAC > TD3 on done triggers due to
   entropy-driven precise reaching.

### Level 2 hypotheses

6. **Deeper networks improve on 6-DOF.** 3-layer MLP > 1-layer MLP
   for PPO/SAC on 6-DOF (diminishing returns beyond 2 layers).

7. **Autograd enables TRPO.** TRPO (natural gradient) > PPO on 6-DOF
   when Fisher vector products are available.

### Level 3 hypotheses

8. **Massive parallelism compensates for sample inefficiency.** 5000
   envs on GPU makes CEM competitive on 6-DOF (sufficient population
   size offsets high dimensionality).

9. **On-policy benefits more from parallelism.** PPO improvement gap
   over SAC shrinks with 5000 envs (on-policy sample inefficiency
   offset by cheap samples).

### Level 4-5 hypotheses

10. **Differentiable physics outperforms model-free RL on known dynamics.**
    Direct trajectory optimization > PPO > SAC when dynamics gradients
    are available (no estimation variance).

11. **Co-design finds better solutions than fixed-morphology RL.**
    Jointly optimized morphology+controller > best controller on a fixed morphology.

Each failed hypothesis is a finding, not a bug.

## Risks

**Over-abstraction.** The Algorithm trait has ONE method. Shared
components are utilities, not mandatory base classes. An algorithm can
ignore everything and implement `train()` from scratch.

**Hand-coded MLP ceiling.** Level 1 maxes out at 1 hidden layer.
This is by design — level 2 (autograd) removes the ceiling. The
trait split ensures no algorithm code changes when the backend upgrades.

**6-DOF arm physics.** More joints = harder dynamics. Mitigated by
verifying the arm with a known-good hand-tuned controller before
training algorithms on it.

**SAC/TD3 complexity.** Off-policy methods have many moving parts.
Mitigated by implementing TD3 first (simpler), then SAC builds on
the same infrastructure.

**Premature GPU optimization.** Level 3 is future work. The trait
interfaces are designed not to prevent it, but we don't build GPU
support until we have compelling evidence it's needed (e.g., level 2
results showing 1000-env runs are CPU-bound).
