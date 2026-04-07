# Phase 4: Visual Examples Refactor + TD3/SAC

> **Status**: Complete (all 6 steps done: shared crate + CEM/REINFORCE/PPO refactors + TD3/SAC new examples)
> **Location**: `examples/fundamentals/sim-ml/vec-env/`
> **Parent spec**: `ML_COMPETITION_SPEC.md`, Phase 4
> **Depends on**: Phase 3 (competition tests, all passing)

## Context

Three visual examples exist (#12 CEM, #14 REINFORCE, #15 PPO). All are
fully hand-rolled — no sim-ml-bridge imports. They duplicate ~26% of
their code (MJCF, VecEnv setup, scene spawning, sync systems, app
scaffolding). Phase 4 refactors these to use shared components and adds
TD3 + SAC examples.

Phase 3 finding: at level 0-1, CEM dominates gradient methods on the
6-DOF task. The visual examples should showcase this reality — CEM as
the crown jewel, gradient methods as "why autograd matters" motivation.

## Design principles

### What to share

Replace hand-rolled duplicates with sim-ml-bridge imports:

| Hand-rolled | Shared component |
|------------|-----------------|
| Inline MJCF + manual VecEnv builder | `reaching_2dof()` → `task.build_vec_env(n_envs)` |
| `apply_policy(obs, params)` | `LinearPolicy::forward(obs)` |
| Custom `AdamState` + `adam_update()` | `OptimizerConfig::adam(lr).build(n_params)` |
| Custom `compute_gae()` (PPO) | `sim_ml_bridge::compute_gae()` |
| Custom `Trajectory` struct | `sim_ml_bridge::Trajectory` from `collect_episodic_rollout()` |
| Custom `randn()` | `rand_distr::Normal` (added to workspace) |
| N/A (new) | `ReplayBuffer` for TD3/SAC |
| N/A (new) | `soft_update()` for TD3/SAC target networks |

### Pitfalls (from stress-testing against the codebase)

**1. Optimizer → Policy sync.** The crate's `Optimizer` trait owns its
own copy of params. After every `optimizer.step(&gradient, ascent)`,
you MUST sync back:
```rust
optimizer.step(&gradient, true);
policy.set_params(optimizer.params());  // REQUIRED — forgetting this = silent bug
```
The hand-rolled examples modify params in-place (no sync needed). The
trait-based optimizer does not. Every example must get this right.
PPO/TD3/SAC have multiple optimizer-network pairs — multiple sync points.

**2. PPO needs extra per-step data.** The shared `Trajectory` struct
has `obs`, `actions`, `rewards`, `done`, `terminal_obs`. PPO also needs
`mu_old` (policy mean at collection time) and `v_old` (value estimate)
per step for importance sampling and GAE. These must be stored in
parallel `Vec`s in the PPO Resource alongside the `Trajectory`, not
added to the shared struct.

**3. `LinearPolicy::forward()` returns `Vec<f64>`, not `[f32; N]`.**
The hand-rolled `apply_policy()` returns `[f32; 2]`. The shared version
returns `Vec<f64>`. Callers need a trivial cast when building action
tensors. Not a problem — just a type change to be aware of.

### What stays inline

Each example keeps its own:

- **State machine** (`Phase::Running` / `Phase::Updating`) — ~30 lines,
  not worth abstracting. A beginner reads one file, top to bottom.
- **Algorithm-specific update logic** — CEM elite selection, REINFORCE
  returns, PPO clipping, TD3 delayed update, SAC entropy. This is what
  the example teaches.
- **Bevy Resource struct** — algorithm-specific fields differ too much.
- **HUD content** — each algorithm shows different metrics.

### What to extract as shared Bevy helper

The Bevy scaffolding for "50 reaching arms in a row" is ~150 lines
duplicated across all examples:

- Model loading from TaskConfig
- PhysicsScenes creation (50 scenes, lane spacing)
- Arm geometry spawning (upper_geom blue, forearm_geom orange)
- Target sphere spawning (green)
- Camera + lighting
- `sync_vec_env_to_scenes` system

Extract this into a shared workspace member crate at
`examples/fundamentals/sim-ml/shared/` (`example-ml-shared`). Each
example adds `example-ml-shared = { path = "../shared" }` to its
dependencies. This follows the existing pattern — each example is
already a separate workspace member with its own `Cargo.toml`.

Proposed API:

```rust
/// Shared setup for 50-arm reaching experiments.
///
/// Returns (VecEnv, PhysicsScenes) after spawning all geometry.
/// The caller wraps these in their algorithm-specific Resource.
pub fn setup_reaching_arms(
    task: &TaskConfig,
    n_envs: usize,
    commands: &mut Commands,
    meshes: &mut ResMut<Assets<Mesh>>,
    materials: &mut ResMut<Assets<StandardMaterial>>,
) -> (VecEnv, PhysicsScenes) {
    // 1. Build VecEnv from TaskConfig
    // 2. Get Model via env.model()
    // 3. Create PhysicsScenes (n_envs scenes, SPACING apart)
    // 4. Spawn arm geoms (blue upper, orange forearm) per scene
    // 5. Spawn target spheres (green) per env
    // 6. Spawn camera + directional light
    // Return (env, scenes)
}

/// Sync VecEnv batch data → PhysicsScenes each frame.
pub fn sync_vec_env_to_scenes(batch: &BatchSim, scenes: &mut PhysicsScenes) {
    sync_batch_geoms(batch, scenes);
}
```

Each example's `setup()` becomes:

```rust
fn setup(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, ...) {
    let task = reaching_2dof();
    let (vec_env, scenes) = setup_reaching_arms(
        &task, 50, &mut commands, &mut meshes, &mut materials,
    );
    commands.insert_resource(scenes);
    commands.insert_resource(MyAlgorithmResource {
        vec_env,
        policy: LinearPolicy::new(task.obs_dim(), task.act_dim(), task.obs_scale()),
        optimizer: OptimizerConfig::adam(0.05).build(policy.n_params()),
        // ... algorithm-specific fields ...
    });
}
```

## The five examples

All use `reaching_2dof()` (2-DOF, linear policies). The 2-DOF task is
where visual learning is most dramatic — arms visibly converge on the
target. 6-DOF is for headless competition tests.

### #12 CEM (refactor existing)

**What changes:**
- Remove inline MJCF → `reaching_2dof()`
- Remove hand-rolled policy eval → `LinearPolicy::forward()`
- Remove hand-rolled `randn()` → `rand_distr::Normal` (added to workspace)
- Use `setup_reaching_arms()` for Bevy scaffolding
- Keep: CEM elite selection logic, Phase state machine, HUD
- **Done** — 8/8 tests pass, identical convergence (20/50 reached gen 25)

**Algorithm-specific Resource fields:**
- `policy: LinearPolicy`
- `mu: Vec<f64>` (elite mean)
- `sigma: Vec<f64>` (perturbation std)
- `perturbations: Vec<Vec<f64>>` (one per env)

**Visual story:** 50 arms, random at first. Each generation, the best
performers (green highlights?) set the mean for the next generation.
Arms converge and reach the target. CEM is the crown jewel — most
dones, best reward at level 0-1.

### #14 REINFORCE (refactor existing)

**What changes:**
- Remove inline MJCF → `reaching_2dof()`
- Remove hand-rolled policy eval → `LinearPolicy::forward()`
- Remove hand-rolled Adam → `OptimizerConfig::adam(0.05).build(n)`
- Remove hand-rolled trajectory struct → use `Trajectory` from rollout
- Use `setup_reaching_arms()` for Bevy scaffolding
- Keep: discounted returns, policy gradient computation, Phase state machine
- **Done** — 7/7 tests pass, 90% reward improvement, 0/50 reached (expected)
- Also: `Optimizer` trait fixed to require `Send + Sync` (foundational fix)

**Algorithm-specific Resource fields:**
- `policy: LinearPolicy`
- `optimizer: Box<dyn Optimizer>`
- `sigma: f64` (exploration noise, decaying)
- `trajectories: Vec<Trajectory>`

**Visual story:** All 50 arms move in unison (same policy). Early
epochs: wild swings. Later: coordinated motion toward target but
can't quite reach — the gradient is too noisy for precise control.
Contrast with CEM which reaches.

### #15 PPO (refactor existing)

**What changes:**
- Remove inline MJCF → `reaching_2dof()`
- Remove hand-rolled policy/value eval → `LinearPolicy`, `LinearValue`
- Remove hand-rolled Adam → two `OptimizerConfig::adam().build()`
- Remove hand-rolled GAE → `sim_ml_bridge::compute_gae()`
- Use `setup_reaching_arms()` for Bevy scaffolding
- Keep: K-pass clipped surrogate, advantage normalization, Phase state machine
- **Done** — 9/9 tests pass, 88% reward improvement, value loss 10072→182

**Algorithm-specific Resource fields:**
- `policy: LinearPolicy`
- `value_fn: LinearValue`
- `policy_optimizer: Box<dyn Optimizer>`
- `value_optimizer: Box<dyn Optimizer>`
- `sigma: f64`
- `trajectories: Vec<Trajectory>` + parallel `mu_old`/`v_old` Vecs (Pitfall #2)

**Visual story:** Similar to REINFORCE but smoother convergence.
The learned baseline (V(s)) reduces gradient variance. HUD shows
value loss and clip fraction — the metrics PPO adds over REINFORCE.

### #16 TD3 (new)

**Pattern B — step-level off-policy.** Different rhythm from CEM/
REINFORCE/PPO: update per step, not per epoch.

**State machine:**
```
Phase::Warmup    → random actions, fill replay buffer
Phase::Running   → step envs, store transitions, sample batch, update
Phase::Paused    → brief pause between display epochs for HUD readability
```

**Resource fields:**
- `policy: LinearPolicy`
- `target_policy: LinearPolicy`
- `q1, q2: LinearQ`
- `target_q1, target_q2: LinearQ`
- `policy_optimizer: Box<dyn Optimizer>`
- `q1_optimizer, q2_optimizer: Box<dyn Optimizer>`
- `replay_buffer: ReplayBuffer`
- `exploration_noise: f64`
- `step_count: usize` (for delayed policy update)

**Algorithm-specific logic (inline):**
1. Step all envs with `policy.forward(obs) + noise`
2. Store transitions in `ReplayBuffer`
3. Sample batch, compute TD targets, update Q1/Q2
4. Every `policy_delay` steps: update policy via `forward_vjp`, soft-update targets

**Visual story:** Warmup phase — arms move randomly (visibly
different from CEM/REINFORCE which start with a policy). Then learning
kicks in — arms gradually coordinate. Off-policy means each transition
is reused ~100x from the buffer. HUD shows buffer size, Q-values.

### #17 SAC (new)

**Pattern B — step-level off-policy with entropy.**

**State machine:** Same as TD3 (Warmup → Running → Paused).

**Resource fields:**
- `policy: LinearStochasticPolicy`
- `q1, q2: LinearQ`
- `target_q1, target_q2: LinearQ`
- `policy_optimizer: Box<dyn Optimizer>`
- `q1_optimizer, q2_optimizer: Box<dyn Optimizer>`
- `replay_buffer: ReplayBuffer`
- `log_alpha: f64` (entropy temperature, auto-tuned)
- `alpha_optimizer: Box<dyn Optimizer>`

**Algorithm-specific logic (inline):**
1. Step all envs with stochastic policy (reparameterized sampling)
2. Store transitions in `ReplayBuffer`
3. Sample batch, compute TD targets with entropy bonus, update Q1/Q2
4. Update policy via reparameterization trick
5. Auto-tune alpha toward target entropy

**Visual story:** Similar to TD3 but with more exploration early on
(entropy regularization drives diverse behavior). HUD shows alpha
(entropy temperature) decreasing as the policy becomes more confident.
Compare: TD3's exploration is fixed noise, SAC's is learned.

## Implementation order

1. **Shared helper module** — `setup_reaching_arms()` + sync system.
   Test by running existing CEM example with the helper before
   refactoring the algorithm logic.

2. **CEM refactor (#12)** — simplest algorithm, proves the pattern.
   Replace hand-rolled components with sim-ml-bridge imports. Verify
   visuals match the existing behavior (same seed → same arm motion).

3. **REINFORCE refactor (#14)** — second on-policy method. Same
   pattern as CEM refactor but adds optimizer import.

4. **PPO refactor (#15)** — adds GAE import, dual optimizer. Most
   complex on-policy refactor.

5. **TD3 new (#16)** — first off-policy visual example. New Pattern B
   state machine. Uses ReplayBuffer, soft_update, forward_vjp.

6. **SAC new (#17)** — builds on TD3 pattern. Adds
   LinearStochasticPolicy, alpha auto-tuning, entropy in HUD.

Each step: build, run visually, verify, commit. One at a time per the
user's review preference.

## Validation

Each example gets:
- Visual check: arms converge, HUD shows sensible metrics
- Headless stress test: `#[cfg(test)]` module with validation checks
- README: museum-plaque style, one concept per example

Cross-reference with competition test results:
- CEM should be the best visual performer (most dones)
- REINFORCE/PPO should show improvement but fewer/no dones
- TD3/SAC should be competitive with CEM on reward

## Files to create/modify

| File | Action |
|------|--------|
| `examples/fundamentals/sim-ml/shared/` | **New** — shared Bevy helper module |
| `examples/fundamentals/sim-ml/vec-env/auto-reset/` | Refactor CEM |
| `examples/fundamentals/sim-ml/vec-env/reinforce/` | Refactor REINFORCE |
| `examples/fundamentals/sim-ml/vec-env/ppo/` | Refactor PPO |
| `examples/fundamentals/sim-ml/vec-env/td3/` | **New** — TD3 visual example |
| `examples/fundamentals/sim-ml/vec-env/sac/` | **New** — SAC visual example |
| `examples/fundamentals/sim-ml/SIM_ML_EXAMPLES_SPEC.md` | Update with #16, #17 |
