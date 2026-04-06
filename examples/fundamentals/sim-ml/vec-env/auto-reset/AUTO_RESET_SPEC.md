# Auto-Reset Spec — Reaching Arm + CEM

> **Status**: Implemented
> **Pattern**: C (VecEnv + PhysicsScenes)

## Concept

50 two-link arms learn to reach a target. Each generation, the 50 envs run
50 perturbations of a shared linear policy. Arms that reach the target
trigger `done` → auto-reset. Arms that time out trigger `truncated` →
auto-reset. After all 50 finish, CEM updates the policy toward the
best-performing perturbations, `reset_all()`, and the next generation begins.

The visual story: generation 1 is pure chaos (arms flailing). By generation
10–15, arms reach roughly the right direction. By generation 25–30, 18–23
of 50 reach smoothly and park at the target. The audience watches a
population collectively learn a coordinated movement.

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
| 12 | Reset feels natural | Arm returns to horizontal rest position |
| 13 | 50 envs as a crowd | Wave of convergence across the row |
| 14 | Reward drives convergence | CEM elite selection = reward-driven |
| 15 | Beginner says "it's learning" | Within 60 seconds |
| 16 | Cool to watch even once | A reaching arm is satisfying |
| 17 | Want to keep watching | "Will they all get it?" |

## Physics

Two-link planar arm in the XZ plane (vertical, with gravity). Shoulder at
the origin, arm hangs down at rest under gravity. Target is up and to the
right — the arm must swing against gravity to reach it.

- Upper arm: L₁ = 0.5 m, 0.5 kg
- Forearm: L₂ = 0.4 m, 0.3 kg
- Total reach: 0.9 m
- Shoulder: hinge, axis -Y, range ±π, damping 2.0
- Elbow: hinge, axis -Y, range ±2.6 rad, damping 1.0
- Motors: gear 10/5, ctrllimited [-1, 1]
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
             limited="true" range="-3.14159 3.14159" damping="2.0"/>
      <geom name="upper_geom" type="capsule" fromto="0 0 0 0.5 0 0"
            size="0.03" mass="0.5" rgba="0.3 0.5 0.85 1"/>
      <body name="forearm" pos="0.5 0 0">
        <joint name="elbow" type="hinge" axis="0 -1 0"
               limited="true" range="-2.6 2.6" damping="1.0"/>
        <geom name="forearm_geom" type="capsule" fromto="0 0 0 0.4 0 0"
              size="0.025" mass="0.3" rgba="0.85 0.4 0.2 1"/>
        <site name="fingertip" pos="0.4 0 0" size="0.015"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="shoulder_motor" joint="shoulder" gear="10"
           ctrllimited="true" ctrlrange="-1 1"/>
    <motor name="elbow_motor" joint="elbow" gear="5"
           ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
```

### Why gear=10/5 and damping=2.0/1.0

With gear=10, gravity compensation at the shoulder requires ctrl ≈ 0.25
(gravity torque ≈ 2.5 Nm / gear 10). This is well within CEM's initial
search range (σ=1.0). Higher gear ratios (20, 50) push the required ctrl
toward ~0.05, which CEM with 50 samples can't reliably distinguish from
zero.

Damping 2.0/1.0 gives a settling time of ~1s, ensuring the arm parks at
the target within the 3s episode rather than oscillating.

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
coordinate used by the done function and rendered as a green sphere.

IK solution: **qpos = (-0.242, 1.982)** — shoulder slightly below
horizontal, elbow bent upward. This is the "elbow-up" configuration.

## CEM Algorithm

Cross-Entropy Method with a linear policy. The simplest evolutionary RL
that uses VecEnv the way real training does: N parallel rollouts per
generation, elite selection, distribution update.

### Policy

Linear with observation normalization:
**action = tanh(W · obs_scaled + b)**

- obs: qpos[0], qpos[1], qvel[0], qvel[1] — **4 dims**
- obs_scaled: obs ⊙ OBS_SCALE — normalizes to ~[-1, 1]
- OBS_SCALE: [1/π, 1/π, 0.1, 0.1]
- action: ctrl[0], ctrl[1] — **2 dims**
- W: [2×4] = 8 parameters
- b: [2] = 2 parameters
- Total: **10 parameters**
- tanh keeps actions in [-1, 1] (motor ctrlrange)

### Why observation normalization

Without normalization, qvel values (~10 rad/s during swings) multiply with
CEM-sampled weights (σ=1.0), producing pre-tanh values of ~10 that fully
saturate. The policy becomes bang-bang: all perturbations output ±1 and CEM
can't distinguish between them. Normalizing obs to [-1, 1] keeps pre-tanh
values moderate (~1–3), preserving gradient information for CEM selection.

### Why obs = joint state only (no target position)

The target is fixed. The policy doesn't need to generalize to different
targets — it just needs to find the right torques for this one. Including
the target in obs would add 2-3 dims of constant input, which CEM can learn
as bias anyway. Keeping obs at 4 dims means 10 parameters, which CEM can
evolve in ~30 generations with 50 samples.

### Distribution

- μ ∈ ℝ¹⁰ — mean of parameter vector (init: zeros)
- σ ∈ ℝ¹⁰ — per-parameter std dev (init: 1.0)
- Sampling: θᵢ = μ + σ ⊙ εᵢ, where εᵢ ~ N(0, I)

### Update (per generation)

1. Run 50 episodes (one per env), each with a different θᵢ
2. Collect cumulative rewards R₁ … R₅₀
3. Sort by reward, take top K = 15 (elite)
4. μ ← mean(elite θs)
5. σ ← max(std(elite θs), σ_min) — floor at 0.1 to prevent collapse

### Why 50 envs and K = 15

CEM theory says N ≥ dim for convergence. In practice, this 2-DOF reaching
problem requires N ≈ 5× dim because the policy must simultaneously learn
correct weight signs (PD stability), weight magnitudes, and bias values
(gravity compensation). With 10 envs, CEM converges 0/5 seeds. With 50
envs, it converges 5/5 seeds (18–23 reaches by generation 30).

Top-15 elites (30%) balances exploitation with exploration. Standard CEM
practice for this scale.

## Reward

Joint-space squared error: **-(qpos[0] - target_q[0])² - (qpos[1] - target_q[1])²**

where target_q = (-0.242, 1.982) is the IK solution for the end-effector
target (0.4, 0, 0.3).

### Why joint-space instead of Cartesian distance

Cartesian distance (`-dist(tip, target)`) creates a reward landscape that
CEM can't navigate efficiently: the arm gets to ~27cm average distance but
can't refine further because the Cartesian gradient at that scale is too
flat relative to CEM's exploration noise. Joint-space reward provides a
direct, convex objective that CEM can optimize.

The **done condition** still uses Cartesian end-effector distance (within
5cm + velocity < 0.5 rad/s), preserving the visual story of "reaching the
green sphere."

## VecEnv Configuration

```rust
const TARGET: [f64; 3] = [0.4, 0.0, 0.3];
const TARGET_QPOS: [f64; 2] = [-0.242, 1.982];
const REACH_THRESHOLD: f64 = 0.05;  // 5 cm
const VEL_THRESHOLD: f64 = 0.5;     // rad/s
const EPISODE_TIMEOUT: f64 = 3.0;   // seconds

VecEnv::builder(model.clone(), NUM_ENVS)
    .observation_space(obs)   // all_qpos() + all_qvel() → dim=4
    .action_space(act)        // all_ctrl() → dim=2
    .reward(move |_m, d| {
        let e0 = d.qpos[0] - TARGET_QPOS[0];
        let e1 = d.qpos[1] - TARGET_QPOS[1];
        -(e0 * e0 + e1 * e1)
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

## Generation Loop

```
State machine:

  RUNNING
    - Step VecEnv each tick (accumulator-based, up to 200/frame)
    - For each env: apply action = tanh(W_i · obs_scaled + b_i) where
      (W_i, b_i) is env i's perturbed policy
    - Track per-env cumulative reward
    - When done[i] or truncated[i]: mark env i as "generation-complete",
      record its cumulative reward
    - Auto-reset fires (env goes back to rest position, keeps running)
    - Ignore subsequent episodes from generation-complete envs
    - When all 50 are generation-complete → transition to UPDATING

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
    let acts = apply_policy(&perturbations[i], obs); // tanh(W·obs_scaled+b)
    actions[i] = acts;
}
```

## Rendering

**Pattern C** (PhysicsScenes): one scene per env, spaced 0.8 m apart along
X axis. `sync_batch_geoms` + `sync_scene_geom_transforms` for pose updates.

**Target spheres**: One green semi-transparent sphere per lane at the target
world position. Bevy entities, not physics objects.

**Arm colors**:
- Upper arm: steel blue (0.3, 0.5, 0.85)
- Forearm: warm orange (0.85, 0.4, 0.2)
- No color changes based on status — the motion tells the story

**Camera**: Front view (looking along +Y), centered on the row of arms.
Distance ~35 m to see all 50. Orbit camera for zoom/pan.

## HUD

```
Reaching Arm (VecEnv + CEM)
  generation: 7 / 30
  phase: RUNNING  (or UPDATING (1.2s))
  reached: 12/50  done: 38/50
  best reward: -62.3
  σ mean: 0.18

Per-Env (sample)
  ★  3 R=  -42.1  DONE
  ★  7 R=  -48.3  DONE
  ★ 22 R=  -51.0  DONE
    15 R=  -89.2  TRUNC
    ...
```

Top-8 envs shown sorted by reward. Star (★) marks elites. Lines update
as envs complete, giving a live scoreboard feel.

## Validation (at generation 25)

1. **Convergence** — at least 10/50 envs trigger `done` (not truncated)
   in the current generation
2. **Reward improvement** — generation 25 best reward > generation 1 best
   reward by at least 50%
3. **Policy mean shifted** — ‖μ‖ > 0.5 (policy learned something, not
   still at zero)
4. **σ decreased** — mean(σ) < 0.5 (distribution tightened from init=1.0)

## Constants

```rust
const NUM_ENVS: usize = 50;
const SPACING: f32 = 0.8;
const TARGET: [f64; 3] = [0.4, 0.0, 0.3];
const TARGET_QPOS: [f64; 2] = [-0.242, 1.982];
const REACH_THRESHOLD: f64 = 0.05;
const VEL_THRESHOLD: f64 = 0.5;
const EPISODE_TIMEOUT: f64 = 3.0;
const PAUSE_TIME: f64 = 1.5;
const NUM_ELITES: usize = 15;
const SIGMA_INIT: f64 = 1.0;
const SIGMA_MIN: f64 = 0.1;
const MAX_GENERATIONS: usize = 30;
const VALIDATION_GEN: usize = 25;
const NUM_PARAMS: usize = 10;
const OBS_SCALE: [f64; 4] = [1/π, 1/π, 0.1, 0.1];
```

## Assumptions Verified

All 8 assumptions were verified via headless tests before implementation.

1. **site_xpos available after forward()** — ✅ tip at (0.9, 0, 0) at
   qpos=(0,0). Used in done condition closure.

2. **Rest position = qpos (0, 0)** — ✅ Data::reset() restores initial
   joint positions.

3. **Gravity makes it fall** — ✅ fingertip z decreases >0.1m in 1s.

4. **CEM converges in 30 generations** — ✅ 23/50 reached with seed=42.
   Reward improves >80% from gen 1.

5. **50 envs sufficient for 10 parameters** — ✅ 3/3 seeds converge
   (18–23 reaches). Original assumption of 10 envs failed — CEM needs
   ~5× the parameter count for this problem.

6. **tanh saturation with σ_init=1.0** — ✅ gen1 reward variance >6M.
   Obs normalization keeps pre-tanh values moderate.

7. **sub_steps=5 gives 100 Hz control** — ✅ time advances 0.01s per step.

8. **Episode timeout 3.0s has enough range** — ✅ reward range >8500
   across 30 generations.
