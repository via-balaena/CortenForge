# Auto-Reset Spec — Reaching Arm + CEM

> **Status**: Draft
> **Pattern**: C (VecEnv + PhysicsScenes)

## Concept

10 two-link arms learn to reach a target. Each generation, the 10 envs run
10 perturbations of a shared linear policy. Arms that reach the target
trigger `done` → auto-reset. Arms that time out trigger `truncated` →
auto-reset. After all 10 finish, CEM updates the policy toward the
best-performing perturbations, `reset_all()`, and the next generation begins.

The visual story: generation 1 is pure chaos (arms flailing). By generation
5–8, arms reach roughly the right direction. By generation 15–20, all 10
reach smoothly and park at the target. The audience watches a population
collectively learn a coordinated movement.

## Why This Example

It hits every VecEnv feature:

| # | Box | How |
|---|-----|-----|
| 1 | `done_fn` meaningful | End-effector at target + low velocity |
| 2 | `truncated_fn` meaningful | Timeout (arm never reached) |
| 3 | `reward_fn` drives learning | CEM selects elites by cumulative reward |
| 4 | `terminal_observations` used | Read to confirm reach at episode end |
| 5 | `observations` closed-loop | Policy reads obs every action step |
| 6 | `action_space` per-step | 2 torques change every action step |
| 7 | `sub_steps` meaningful | Hold torques for 5 physics steps (100 Hz) |
| 8 | Async episode lengths | Good arms finish early, bad arms timeout |
| 9 | Motion is the content | You watch arms swing through full trajectories |
| 10 | Good/bad obvious mid-episode | Smooth reach vs wild flail |
| 11 | Chaos → order without HUD | Visible in the motion alone |
| 12 | Reset feels natural | Arm returns to hanging rest position |
| 13 | 10 envs readable | Side by side, same plane |
| 14 | Reward drives convergence | CEM elite selection = reward-driven |
| 15 | Beginner says "it's learning" | Within 30 seconds |
| 16 | Cool to watch even once | A reaching arm is satisfying |
| 17 | Want to keep watching | "Will they all get it?" |

## Physics

Two-link planar arm in the XZ plane (vertical, with gravity). Shoulder at
the origin, arm hangs down at rest under gravity. Target is up and to the
right — the arm must swing against gravity to reach it.

- Upper arm: L₁ = 0.5 m, 0.5 kg
- Forearm: L₂ = 0.4 m, 0.3 kg
- Total reach: 0.9 m
- Shoulder: hinge, axis -Y, range ±π, damping 0.5
- Elbow: hinge, axis -Y, range ±2.6 rad, damping 0.3
- Motors: gear 20/10, ctrllimited [-1, 1]
- Fingertip: `<site>` at end of forearm for position tracking
- No contact, RK4 integrator, dt = 0.002

### MJCF

```xml
<mujoco model="reaching-arm">
  <compiler angle="radian" inertiafromgeom="true"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <body name="upper_arm" pos="0 0 0">
      <joint name="shoulder" type="hinge" axis="0 -1 0"
             limited="true" range="-3.14159 3.14159" damping="0.5"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.5 0 0"
            size="0.03" mass="0.5" rgba="0.3 0.5 0.85 1"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow" type="hinge" axis="0 -1 0"
               limited="true" range="-2.6 2.6" damping="0.3"/>
        <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.4 0 0"
              size="0.025" mass="0.3" rgba="0.85 0.4 0.2 1"/>
        <site name="fingertip" pos="0.4 0 0" size="0.015"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="20"
           ctrllimited="true" ctrlrange="-1 1"/>
    <motor name="elbow_motor" joint="elbow" gear="10"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
```

### Rest Position

At qpos = (0, 0), both links point along +X (horizontal right). Under
gravity (-Z), the arm swings down and settles with the shoulder near -π/2
(hanging straight down). `Data::reset()` restores qpos = (0, 0), so each
episode starts with the arm horizontal — it immediately begins to fall,
which gives the policy something to work with from the first step.

### Target

Fixed world position: **(0.4, 0, 0.3)** — up and to the right of the
shoulder. Within the workspace (distance ≈ 0.5 m < 0.9 m reach), requiring
the arm to hold against gravity. Not a physics object — just a world-frame
coordinate used by reward/done functions and rendered as a green sphere.

## CEM Algorithm

Cross-Entropy Method with a linear policy. The simplest evolutionary RL
that uses VecEnv the way real training does: N parallel rollouts per
generation, elite selection, distribution update.

### Policy

Linear: **action = tanh(W · obs + b)**

- obs: qpos[0], qpos[1], qvel[0], qvel[1] — **4 dims**
- action: ctrl[0], ctrl[1] — **2 dims**
- W: [2×4] = 8 parameters
- b: [2] = 2 parameters
- Total: **10 parameters**
- tanh keeps actions in [-1, 1] (motor ctrlrange)

### Why obs = joint state only (no target position)

The target is fixed. The policy doesn't need to generalize to different
targets — it just needs to find the right torques for this one. Including
the target in obs would add 2-3 dims of constant input, which CEM can learn
as bias anyway. Keeping obs at 4 dims means 10 parameters, which CEM can
evolve in ~20–30 generations with 10 samples.

### Distribution

- μ ∈ ℝ¹⁰ — mean of parameter vector (init: zeros)
- σ ∈ ℝ¹⁰ — per-parameter std dev (init: 1.0)
- Sampling: θᵢ = μ + σ ⊙ εᵢ, where εᵢ ~ N(0, I)

### Update (per generation)

1. Run 10 episodes (one per env), each with a different θᵢ
2. Collect cumulative rewards R₁ … R₁₀
3. Sort by reward, take top K = 3 (elite)
4. μ ← mean(elite θs)
5. σ ← std(elite θs) + σ_min (floor at 0.05 to prevent collapse)

### Why K = 3

With 10 samples and 10 parameters, top-3 elites (~30%) balances
exploitation (sharp update) with exploration (enough diversity). Standard
CEM practice for this scale.

## VecEnv Configuration

```rust
const TARGET: [f64; 3] = [0.4, 0.0, 0.3];
const REACH_THRESHOLD: f64 = 0.05;  // 5 cm
const VEL_THRESHOLD: f64 = 0.3;     // rad/s
const EPISODE_TIMEOUT: f64 = 3.0;   // seconds

VecEnv::builder(model.clone(), NUM_ENVS)
    .observation_space(obs)   // all_qpos() + all_qvel() → dim=4
    .action_space(act)        // all_ctrl() → dim=2
    .reward(move |_m, d| {
        let tip = d.site_xpos[0];  // fingertip site
        let dx = tip.x - TARGET[0];
        let dz = tip.z - TARGET[2];
        let dist = (dx*dx + dz*dz).sqrt();
        let vel = (d.qvel[0]*d.qvel[0] + d.qvel[1]*d.qvel[1]).sqrt();
        -(dist + 0.1 * vel)
    })
    .done(move |_m, d| {
        let tip = d.site_xpos[0];
        let dx = tip.x - TARGET[0];
        let dz = tip.z - TARGET[2];
        let dist = (dx*dx + dz*dz).sqrt();
        let vel = (d.qvel[0]*d.qvel[0] + d.qvel[1]*d.qvel[1]).sqrt();
        dist < REACH_THRESHOLD && vel < VEL_THRESHOLD
    })
    .truncated(|_m, d| d.time > EPISODE_TIMEOUT)
    .sub_steps(5)
    .build()
```

### Reward breakdown

`-(distance + 0.1 × velocity)` per step, accumulated over the episode.

- Distance: how far the fingertip is from the target
- Velocity penalty: encourages smooth arrival, not wild swinging through
- Accumulated: episodes that reach fast get fewer negative-reward steps →
  higher total reward. CEM naturally selects for both accuracy and speed.

## Generation Loop

```
State machine:

  RUNNING
    - Step VecEnv each tick (accumulator-based, up to 200/frame)
    - For each env: apply action = tanh(W_i · obs + b_i) where (W_i, b_i)
      is env i's perturbed policy
    - Track per-env cumulative reward
    - When done[i] or truncated[i]: mark env i as "generation-complete",
      record its cumulative reward
    - Auto-reset fires (env goes back to rest position, keeps running)
    - Ignore subsequent episodes from generation-complete envs
    - When all 10 are generation-complete → transition to UPDATING

  UPDATING
    - Stop stepping (freeze all envs for PAUSE_TIME = 1.5 s)
    - CEM update: sort rewards, elite selection, update μ and σ
    - After pause: reset_all(), sample new perturbations → RUNNING
```

The pause between generations is honest — it represents the policy update
computation. In real RL training, there IS a moment between rollout
collection and gradient/evolution update. We're making it visible.

### Action application

Each action step (every 5 physics steps = every 10 ms):

```rust
for i in 0..NUM_ENVS {
    let obs = result.observations.row(i);  // [qpos0, qpos1, qvel0, qvel1]
    let (w, b) = &perturbations[i];        // this env's perturbed policy
    let raw = w * obs + b;                 // [2] raw output
    actions[i] = [tanh(raw[0]), tanh(raw[1])];
}
```

## Rendering

**Pattern C** (PhysicsScenes): one scene per env, spaced 2.0 m apart along
Y axis. `sync_batch_geoms` + `sync_scene_geom_transforms` for pose updates.

**Target spheres**: One green semi-transparent sphere per lane at the target
world position. Bevy entities, not physics objects.

**Arm colors**:
- Upper arm: steel blue (0.3, 0.5, 0.85)
- Forearm: warm orange (0.85, 0.4, 0.2)
- No color changes based on status — the motion tells the story

**Ground reference**: Optional thin line or plane at the shoulder height to
give vertical reference.

**Camera**: Front view (looking along +Y), centered on the row of arms.
Far enough back to see all 10.

## HUD

```
Reaching Arm (VecEnv + CEM)
  generation: 7 / 30
  phase: RUNNING  (or UPDATING)
  reached: 4/10
  best reward: -42.3
  σ mean: 0.18

Per-Env
   0 R=-180  3.0s  TRUNC
   1 R= -89  2.1s  TRUNC
   2 R= -42  0.8s  DONE   ★
   3 R= -67  1.4s  DONE
   ...
```

Per-env line shows: cumulative reward, episode duration, how it ended.
Star (★) on elites selected for the CEM update. Lines update as envs
complete, giving a live scoreboard feel.

## Validation (at generation 25)

1. **Convergence** — at least 6/10 envs trigger `done` (not truncated)
   in the current generation
2. **Reward improvement** — generation 25 best reward > generation 1 best
   reward by at least 50%
3. **Policy mean shifted** — ‖μ‖ > 0.5 (policy learned something, not
   still at zero)
4. **σ decreased** — mean(σ) < 0.5 (distribution tightened from init=1.0)
5. **terminal_obs populated** — every done/truncated had
   `terminal_observations[i].is_some()`
6. **Auto-reset obs correct** — after reset, qpos near (0, 0)

## Constants

```rust
const NUM_ENVS: usize = 10;
const SPACING: f32 = 2.0;
const TARGET: [f64; 3] = [0.4, 0.0, 0.3];
const REACH_THRESHOLD: f64 = 0.05;
const VEL_THRESHOLD: f64 = 0.3;
const EPISODE_TIMEOUT: f64 = 3.0;
const PAUSE_TIME: f64 = 1.5;
const NUM_ELITES: usize = 3;
const SIGMA_INIT: f64 = 1.0;
const SIGMA_MIN: f64 = 0.05;
const MAX_GENERATIONS: usize = 30;
const VALIDATION_GEN: usize = 25;
```

## Assumptions to Verify

1. **site_xpos available in reward/done closures** — `Data::site_xpos[0]`
   is populated after `forward()` runs inside `step_all()`. Verify with a
   unit test.

2. **Rest position = qpos (0, 0)** — `Data::reset()` restores the MJCF
   initial joint positions. Verify that the arm starts horizontal.

3. **Gravity makes it fall** — with qpos=(0,0), the arm is horizontal.
   Under gravity it should swing down. Verify the motion direction.

4. **CEM converges in 30 generations** — with the given arm parameters,
   target position, and CEM hyperparameters. This needs tuning runs.

5. **10 envs sufficient for 10 parameters** — CEM theory suggests N ≥ dim.
   We're at the minimum. If convergence is unreliable, increase to 15.

6. **tanh saturation** — with σ_init=1.0, early perturbations may produce
   large pre-tanh values that saturate. This is fine (the policy explores
   the full [-1,1] range), but verify CEM can still distinguish gradients.

7. **sub_steps=5 gives 100 Hz control** — dt=0.002, 5 sub-steps = 10 ms
   per action. Verify this is fast enough for smooth reaching (not so fast
   that the policy has too many decisions to make per episode).

8. **Episode timeout 3.0 s with sub_steps=5** — that's 300 action steps
   per episode. Verify CEM cumulative reward has enough dynamic range to
   distinguish good from bad.
