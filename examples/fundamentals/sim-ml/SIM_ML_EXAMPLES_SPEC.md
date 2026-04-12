# sim-ml-bridge Visual Examples Spec

> **Status**: Draft v5 — Phases 1–4 complete, Phases 5–8 specified
> **Location**: `examples/fundamentals/sim-ml/`
> **Depends on**: `sim-ml-bridge`, `sim-bevy`, `sim-core`, `sim-mjcf`

## Context

The sim-ml-bridge crate (Phases 1–6 complete, 342 tests) needs visual Bevy
examples following the same "baby steps, one concept per example" pattern as
`examples/fundamentals/sim-cpu/`. The crown jewels should have "watch the
learning happen" quality — a population of arms going from chaos to
coordinated reaching.

Phases 1–4 covered spaces, SimEnv, VecEnv, and the 2-DOF algorithm ladder
(all 5 algorithms with hand-coded linear policies). Since then the crate has
grown substantially — MLP networks, autograd (tape-based reverse-mode AD),
6-DOF and obstacle tasks, full checkpoint resume, best-policy tracking, and
a competition runner — all proven by 342 headless tests but with zero visual
Bevy proof. Phases 5–8 close that gap.

## Prerequisite sim-cpu Examples

All sim-ml examples build on features confirmed working in Track 1A + 1B
(through subdirectory 14, `mesh-collision/`). Specifically:

| sim-ml example | Depends on sim-cpu feature | Status |
|---|---|---|
| All Phase 1+2 (pendulum) | hinge joint, motor actuator, jointpos/jointvel sensors | ✅ Track 1A |
| #4 obs-rich (cart-pole) | slide + hinge joints, contact (ground plane) | ✅ Track 1A |
| All Phase 3 (VecEnv) | batch-sim (BatchSim, PhysicsScenes, sync_batch_geoms) | ✅ Track 1B #13 |
| #12 auto-reset (reaching arm) | 2 hinge joints, motor actuators, gravity, site_xpos | ✅ Track 1A |

**No dependency on unfinished Track 1B items** (heightfield, hill muscle,
adhesion, flex bodies, collision pairs, plugins). All prerequisite features
are confirmed working with visual + stress-test validation.

## Existing Headless Crate Examples

The crate already has 3 headless CLI examples in `sim/L0/ml-bridge/examples/`:
- `pendulum_swingup.rs` — SimEnv lifecycle demo
- `cartpole_balance.rs` — sensor-based observation, proportional controller
- `vec_env_throughput.rs` — 64-env throughput measurement

**These stay as quick-test developer demos.** They serve a different purpose
(API smoke test, throughput measurement) than the visual examples (one concept
per example, museum-plaque documentation, validation checks). No overlap —
the visual examples demonstrate concepts; the crate examples demonstrate the
API end-to-end.

## Structure

```
examples/fundamentals/sim-ml/
├── spaces/                   # Phase 1 — DONE
│   ├── stress-test/          #1  — headless correctness gate
│   ├── obs-extract/          #2  — see tensor values update live
│   ├── act-inject/           #3  — action drives the pendulum
│   ├── obs-rich/             #4  — 13 extractor types in one HUD
│   └── act-clamping/         #5  — out-of-range actions get clipped
├── sim-env/                  # Phase 2 — DONE
│   ├── stress-test/          #6  — headless correctness gate
│   ├── episode-loop/         #7  — Pattern B spike + episode lifecycle
│   ├── sub-stepping/         #8  — side-by-side 1 vs 10 sub-steps
│   └── on-reset/             #9  — different starting angle each episode
├── vec-env/                  # Phases 3–4 — DONE (except #13)
│   ├── stress-test/          #10 — headless correctness gate
│   ├── parallel-step/        #11 — 8 pendulums, different actions
│   ├── auto-reset/           #12 — ★ 50 reaching arms + CEM (sampling)
│   ├── terminal-obs/         #13 — done vs truncated, value bootstrapping
│   ├── reinforce/            #14 — ★ 50 reaching arms + REINFORCE (gradient)
│   ├── ppo/                  #15 — ★ 50 reaching arms + PPO (actor-critic)
│   ├── td3/                  #16 — ★ 50 reaching arms + TD3 (off-policy, DPG)
│   └── sac/                  #17 — ★ 50 reaching arms + SAC (off-policy, entropy)
├── persistence/              # Phase 7
│   ├── train-then-replay/    #P1 — CEM train → save artifact → replay (DONE)
│   └── checkpoint-resume/    #24 — full checkpoint save + resume training
├── 6dof/                     # Phase 5
│   ├── stress-test/          #18 — headless 6-DOF integration checks
│   ├── cem-linear/           #19 — ★ 6-DOF + CEM + linear policy
│   ├── ppo-mlp/              #20 — ★ 6-DOF + PPO + MLP (hidden layer)
│   └── sac-autograd/         #21 — ★ 6-DOF + SAC + autograd 2-layer
├── obstacle/                 # Phase 6
│   ├── stress-test/          #22 — headless obstacle edge cases
│   └── sac-autograd/         #23 — ★ 50 arms curving around obstacle
├── competition/              # Phase 8
│   └── dashboard/            #25 — multi-algorithm comparison replay
└── shared/                   # shared Bevy scaffolding
```

25 examples total (17 existing + 8 new). Package naming: `example-ml-{group}-{name}`.

Examples #12, #14–#17 form the **2-DOF algorithm ladder**: same arm,
same target, same VecEnv — five different optimizers, all with hand-coded
linear policies (10–12 params). The visual progression tells the story of
the level 0-1 landscape:

| # | Algorithm | Type | Visual result |
|---|-----------|------|---------------|
| 12 | CEM | Sampling-based | ★ Best performer — arms converge and reach target |
| 14 | REINFORCE | Policy gradient | Arms improve but can't reach precisely |
| 15 | PPO | Actor-critic | Better than REINFORCE, still below CEM |
| 16 | TD3 | Off-policy (DPG) | ~30% improvement then plateau (linear Q saturates) |
| 17 | SAC | Off-policy (entropy) | ~50% steady improvement (entropy prevents saturation) |

**Note on ordering:** the original spec predicted gradient methods would
dominate CEM. Phase 3 competition tests (seed 42, 50ep/50env) showed the
**opposite** at level 0-1: CEM (-1.05, 49 dones) >> TD3/SAC (-11) >>
PPO (-3449) >> REINFORCE (-7500). Hand-coded gradients in 614-dim space
are too noisy to outperform gradient-free search on a smooth quadratic
reward. The visual examples should reflect what actually happens — CEM
as the crown jewel, gradient methods as the "why autograd matters"
motivation. See `COMPETITION_TESTS_SPEC.md` for full analysis.

Examples #19–#21 form the **6-DOF architecture ladder**: same 6-DOF task,
three different (algorithm, network) pairs — each step adds one variable.
The progression tells the story of why deeper networks and automatic
differentiation matter:

| # | Algorithm | Network | One new variable | Visual result |
|---|-----------|---------|------------------|---------------|
| 19 | CEM | Linear | Task complexity (2→6 DOF) | CEM struggles — 12-dim obs, 6-dim action |
| 20 | PPO | MLP (hidden=32) | Network architecture | Gradient method shines with capacity |
| 21 | SAC | Autograd [64,64] ReLU | Autograd + entropy | Deep network at the frontier |

Examples #22–#23 add **obstacle avoidance** — the reward landscape becomes
non-convex. Example #25 is the **competition dashboard** — all 5 algorithms
on 6-DOF, best policies replayed side by side.

## Bevy Integration Patterns

### Pattern A — Spaces (Phase 1)

Standard `PhysicsModel`/`PhysicsData` + `step_physics_realtime`. Call
`obs_space.extract()` or `act_space.apply()` in Bevy systems. Show values
in `PhysicsHud`. No SimEnv/VecEnv involved — demonstrates the bridge types
working directly with the existing sim-bevy infrastructure.

**Proven by:** every sim-cpu single-scene example (motor, clock, etc.).

### Pattern B — SimEnv (Phase 2)

`SimEnv` as a Bevy Resource. Custom stepping system calls `env.step()`.
Copy geom poses from `env.data()` → `PhysicsData` each frame for rendering.
For the sub-stepping side-by-side example, use `PhysicsScenes` (2 scenes).

**This pattern is unproven.** Example #7 (episode-loop) is the spike — its
primary purpose is to validate that the copy-to-PhysicsData approach works
cleanly with `sync_geom_transforms`. If friction emerges (e.g., missing
fields in the copy, ownership issues, performance), we stop and resolve
before proceeding to #8 and #9.

The copy needs at minimum: `geom_xpos`, `geom_xmat`, `time` (for
ValidationHarness), and any sensor data the HUD displays. We'll discover
the exact field set during the spike.

**Fallback if copy-to-PhysicsData doesn't work:** add a
`SimEnv::into_parts() -> (Arc<Model>, Data, ...)` or
`SimEnv::swap_data(&mut self, data: &mut Data)` method so the Bevy
resource system can own the Data directly as `PhysicsData` while SimEnv
borrows it for stepping. This is preferred over the alternative (having
SimEnv accept external `&mut Data`) because it preserves SimEnv's
ownership model for non-Bevy users — the Bevy integration is the special
case, not the default API. We only pursue this if the copy approach fails
during the #7 spike.

**Pattern B + PhysicsScenes (for #8 sub-stepping):** when two SimEnvs need
side-by-side rendering, each SimEnv gets its own `PhysicsScene` via
`scenes.add(label, model.clone(), data)`. After each SimEnv steps, copy
geom poses from `env.data()` into the scene's data. The sync is the same
as Pattern C's `sync_scene_geom_transforms` — the only difference is the
source (SimEnv.data() instead of BatchSim.env(i)).

### Pattern C — VecEnv (Phase 3)

`VecEnv` as a Bevy Resource. `PhysicsScenes` with one scene per env.
`sync_batch_geoms(vec_env.batch(), &mut scenes)` copies poses. Same proven
pattern as `batch-sim/reset-subset`.

**Proven by:** `batch-sim/parameter-sweep` and `batch-sim/reset-subset`.

## Phase 1: Spaces

### #1. stress-test (headless)

The crate's 111 unit tests cover individual API correctness (shapes,
clamping, builder errors, etc.). This stress test covers **integration
scenarios** the unit tests don't:

1. **Realistic model** — cart-pole with contacts, sensors, multiple bodies
   (unit tests use a minimal 1-joint pendulum)
2. **All 13 extractors in one space** — concatenation of every extractor
   type on a single rich model, verifying dim accounting end-to-end
3. **All 5 injectors in one space** — ctrl + qfrc_applied + xfrc_applied
   + mocap_pos + mocap_quat on a mocap-enabled model
4. **1000-step endurance** — extract + apply in a loop for 1000 steps,
   no panics, no NaN, no drift in tensor shapes
5. **Overlapping ranges** — qpos(0..2) + qpos(1..3) produces correct
   concatenation (duplicated elements, not an error)
6. **Empty space** — zero extractors, dim=0, extract produces empty tensor
7. **Round-trip fidelity** — extract obs, modify one element, apply via
   action space, extract again, verify the modification propagated
8. **Batch parity** — extract_batch(N) == N individual extract() calls,
   apply_batch(N) == N individual apply() calls
9. **Builder errors on realistic model** — out-of-bounds on actual nq/nv/nu,
   nonexistent sensor name on a model with real sensors

### #2. obs-extract — "Observation Extraction"

Pendulum swings freely from 45°. HUD shows extracted tensor values
alongside raw Data fields (f64 vs f32 side-by-side).

**MJCF:**
```xml
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body name="pendulum" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0 0 -0.5" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="hinge" name="motor"/>
  </actuator>
  <sensor>
    <jointpos joint="hinge" name="angle"/>
    <jointvel joint="hinge" name="angvel"/>
  </sensor>
</mujoco>
```

**ObservationSpace:** `all_qpos() + all_qvel()` → dim=2.

**Validates:**
- obs.shape() == [2] (qpos + qvel)
- obs[0] matches data.qpos[0] as f32 within f32 epsilon
- obs[1] matches data.qvel[0] as f32
- Two calls to extract() on same Data produce identical tensors

### #3. act-inject — "Action Injection"

Pendulum driven by sinusoidal action through `ActionSpace` (ctrllimited,
ctrlrange=[-2,2]). HUD shows action tensor → ctrl value → actuator force
chain.

**MJCF:** Same as #2 but with `ctrllimited="true" ctrlrange="-2 2"`.

**ActionSpace:** `all_ctrl()` → dim=1.

**Validates:**
- act_space.dim() == 1
- data.ctrl[0] matches f64::from(action_val) after apply
- TensorSpec bounds match model's ctrlrange
- Pendulum oscillates (qvel changes sign multiple times)

### #4. obs-rich — "Rich Observation Space"

Cart-pole model. Single large observation from qpos + qvel + named sensors
+ xpos + energy + time + contact_count. HUD breaks down the obs vector by
source with labels.

**MJCF:**
```xml
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <geom type="plane" size="5 5 0.1"/>
    <body name="cart" pos="0 0 0.5">
      <joint name="cart_slide" type="slide" axis="1 0 0" damping="0.1"/>
      <geom type="box" size="0.2 0.1 0.05" mass="1"/>
      <body name="pole" pos="0 0 0.05">
        <joint name="pole_hinge" type="hinge" axis="0 1 0"/>
        <geom type="capsule" size="0.02" fromto="0 0 0 0 0 0.5" mass="0.1"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor joint="cart_slide" name="force"/>
  </actuator>
  <sensor>
    <jointpos joint="cart_slide" name="cart_pos"/>
    <jointvel joint="cart_slide" name="cart_vel"/>
    <jointpos joint="pole_hinge" name="pole_angle"/>
    <jointvel joint="pole_hinge" name="pole_angvel"/>
  </sensor>
</mujoco>
```

**ObservationSpace:** `all_qpos() + all_qvel() + sensor("cart_pos") +
sensor("cart_vel") + sensor("pole_angle") + sensor("pole_angvel") +
xpos(1..3) + energy() + time() + contact_count()`.

**Validates:**
- obs_dim matches sum of all extractor dims
- Named sensor extraction matches manual sensordata indexing
- xpos extraction has 3 floats per body
- energy extractor produces 2 floats (kinetic, potential)

### #5. act-clamping — "Ctrl Clamping and Action Bounds"

Pendulum with ctrlrange=[-1,1]. Ramping action from -3 to +3 over time.
HUD shows requested vs actual (clamped). Red/green indicator when clamped.

**MJCF:** Same as #2 but with `ctrllimited="true" ctrlrange="-1 1"`.

**ActionSpace:** `all_ctrl()` → dim=1.

**Validates:**
- ctrl never exceeds ctrlrange
- Within-range actions pass through unclamped
- At upper/lower bound: ctrl == exactly ±1.0
- act_space.spec().low/high match ctrlrange

## Phase 2: SimEnv

### #6. stress-test (headless)

Covers integration scenarios beyond the crate's 111 unit tests:

1. **1000-step episode** — step a SimEnv for 1000 steps without panic/NaN
2. **Multi-episode endurance** — run 50 episodes (reset + step until done),
   verify no state leaks between episodes (qpos resets to initial)
3. **Early termination timing** — with sub_steps=100 and low done threshold,
   verify step stops early and time < sub_steps * timestep
4. **on_reset stateful closure** — closure owns a counter, verify it
   increments exactly once per reset across 20 episodes
5. **on_reset + forward() interaction** — set qpos in on_reset, verify
   sensordata reflects the modified qpos (not stale pre-reset values)
6. **data_mut() round-trip** — modify via data_mut(), verify next observe()
   reflects the change, verify next step() runs from modified state
7. **Reward sign under known state** — set qpos to known value via
   data_mut(), step, verify reward matches manual calculation
8. **Done precedence over truncated** — set state that triggers both
   done and truncated simultaneously, verify done=true, truncated=false
9. **Sub-steps=1 vs sub_steps=N parity** — 5 individual steps with
   sub_steps=1 == 1 step with sub_steps=5 (same final state)
10. **observe() == obs_space.extract(env.data())** — consistency check

### #7. episode-loop — "Episode Lifecycle" (Pattern B spike)

**This is the Pattern B spike.** Primary goal: validate that SimEnv as a
Bevy Resource with copy-to-PhysicsData rendering works. Secondary goal:
demonstrate the episode lifecycle.

Pendulum with sinusoidal policy. Done when |qpos| > 2.0. Truncated at
t > 5s. Watch the pendulum reset when it crosses threshold. HUD shows
episode count, step count, cumulative reward, done/truncated flags,
StepResult fields.

**MJCF:** Same as #2.

**Pattern B implementation:** After `env.step()`, copy the minimum fields
from `env.data()` into `PhysicsData` for `sync_geom_transforms`. The exact
field set will be determined during implementation — at minimum `geom_xpos`,
`geom_xmat`, `time`. If the copy is too brittle or expensive, we'll pivot
to an alternative (e.g., SimEnv storing data externally, or a dedicated
sync function).

**Validates:**
- At least 2 episodes completed by t=30s
- After reset, obs matches initial state
- done fires when |qpos| > 2.0
- truncated fires at time limit
- Reward computed correctly

### #8. sub-stepping — "Sub-Stepping and Action Rate"

Two pendulums side-by-side via PhysicsScenes. Left: sub_steps=1 (500Hz
actions). Right: sub_steps=10 (50Hz actions). Same sinusoidal policy.
Visually different responsiveness.

**MJCF:** Same as #2. Two SimEnvs with same model, different sub_steps.

**Validates:**
- Both envs at same sim time
- sub_steps=1 env has 10x more action steps
- Both envs have identical physics step count
- Trajectories diverge (different action rates matter)

### #9. on-reset — "Domain Randomization via on_reset"

Pendulum with zero action. on_reset sets qpos to a cycling sequence of
angles (0.5, 1.0, 1.5, 2.0, 0.5, ...). Falls quickly → done → resets at
new angle. Watch it start from different positions each episode.

**MJCF:** Same as #2. Done when |qpos| > 2.5 (quickly reached from any
starting angle under gravity with no torque).

**Validates:**
- Each episode starts at a different angle
- Sensordata reflects modified qpos (forward() recomputed)
- on_reset called exactly once per reset
- Initial observation reflects randomized state

## Phase 3: VecEnv

### #10. stress-test (headless)

Covers integration scenarios beyond the crate's 111 unit tests:

1. **Bit-exact parity** — VecEnv(N).step() with identical actions matches
   N sequential SimEnv.step() calls (the gold-standard correctness test),
   run for 100 steps not just 1
2. **100-step endurance with auto-resets** — 64 envs, 100 steps, with
   done thresholds that trigger resets during the run. No panics, no NaN.
3. **Auto-reset terminal_obs correctness** — after auto-reset,
   terminal_observations[i] contains pre-reset state, observations[i]
   contains post-reset state. Verified by comparing qpos values.
4. **Diverged env terminal_obs is None** — (if triggerable) diverged env
   gets terminal_observations[i] = None, not garbage state
5. **on_reset env_index correctness** — each env gets its own index,
   verified by on_reset setting qpos = env_idx * 0.01
6. **Sub-stepping: all sub-steps run** — with sub_steps=10, verify time
   advances by 10 * timestep per step (no early exit in VecEnv)
7. **Wrong action shape** — [N-1, dim] and [N, dim+1] both return
   VecStepError::ActionShapeMismatch
8. **Per-env action isolation over 50 steps** — different constant actions
   per env, verify states diverge progressively
9. **Done precedence** — env that triggers both done and truncated
   simultaneously: done=true, truncated=false
10. **Reset-all correctness** — reset_all() returns [N, obs_dim], all
    observations match initial state
11. **VecStepResult field lengths** — rewards.len(), dones.len(),
    truncateds.len(), terminal_observations.len(), errors.len() all == N

### #11. parallel-step — "Parallel Stepping"

8 pendulums, constant actions linearly spaced [-1,+1]. Blue-to-red color
gradient by action value. HUD shows per-env action, qpos, reward, batched
tensor shapes.

**MJCF:** Same as #2 but with `ctrllimited="true" ctrlrange="-1 1"`.

**Validates:**
- observations.shape() == [8, 2]
- Per-env observations differ (different actions)
- rewards.len() == 8
- Bit-exact match vs sequential SimEnv stepping

### #12. auto-reset ★ — "Auto-Reset: Reaching Arm + CEM" — DONE

**Sampling-based learning.** 50 two-link arms learn to reach a target via
CEM. Each generation, VecEnv runs 50 perturbations of a shared linear
policy. Done triggers auto-reset (arm reached target). Truncated triggers
auto-reset (timeout). After all 50 finish, CEM selects top-15 elites,
updates μ and σ, resets all, next generation.

Visual story: gen 1 chaos → gen 10–15 partial convergence (~20/50 reach)
→ gen 25+ steady at 18–22/50. Not all arms converge — each runs a
different perturbation. This is the nature of CEM (sampling-based).

**Physics:** 2-link planar arm, gear=10/5, damping=2.0/1.0, dt=0.002,
RK4, no contact. Fingertip site at end of forearm.

**Policy:** `action = tanh(W · obs_scaled + b)`, 10 params. Obs normalized
via OBS_SCALE to prevent tanh saturation.

**Reward:** Joint-space squared error `-(qpos - target_qpos)²`. Cartesian
distance plateaus at ~27cm; joint-space is directly optimizable by CEM.

**Done:** Cartesian end-effector within 5cm of target AND vel < 0.5 rad/s.

**Key finding from assumption testing:** CEM needs ~5× the parameter count
in envs (50 envs for 10 params). 10 envs produced 0 reaches across all
configurations tested.

Full spec + implementation notes: `auto-reset/AUTO_RESET_SPEC.md`.

### #14. reinforce ★ — "Policy Gradient: Reaching Arm + REINFORCE"

**Gradient-based learning.** Same arm, same target, same VecEnv as #12 —
but replaces CEM with REINFORCE (vanilla policy gradient). All 50 envs
share ONE policy (no perturbations). The visual difference is dramatic:
all 50 arms converge to the same smooth reach in unison.

**Why this matters:** CEM can only rank whole rollouts — it doesn't know
WHICH actions were good. REINFORCE computes per-step credit assignment
via the policy gradient theorem, giving a gradient that points toward
better actions at each state. This is the fundamental advance of modern RL.

**Policy:** Gaussian stochastic: `a ~ N(μ(s), σ²)` where `μ(s) = tanh(W·obs+b)`.
Same 10 params as CEM. σ is a learned or annealed exploration parameter.
During eval (visual), all envs use the mean action — producing identical
behavior.

**Gradient (hand-coded, no autograd):**
```
∇θ J ≈ (1/N) Σᵢ Σₜ ∇θ log π(aₜ|sₜ) · Rₜ

For Gaussian π with linear mean:
  ∇θ log π(a|s) = (a - μ(s)) / σ² · ∇θ μ(s)
  ∇W μ = (1 - tanh²(W·s+b)) · s^T    (outer product)
  ∇b μ = (1 - tanh²(W·s+b))
```

~50 lines of gradient math. No autograd framework needed.

**Training loop:**
```
For each epoch:
  1. reset_all()
  2. Run full episodes, recording (obs, action, reward) per step per env
  3. Compute discounted returns Rₜ = Σ γ^k r_{t+k}
  4. Subtract baseline (mean return) to reduce variance
  5. Compute policy gradient from recorded trajectories
  6. Update W, b via gradient ascent: θ += lr · ∇θ J
```

**VecEnv config:** Identical to #12 (same obs, act, reward, done, truncated,
sub_steps). Only the optimizer differs.

**Visual story:** Epoch 1 — random actions, arms flail. Epoch 5–10 — all
50 arms swing toward target together (not varied like CEM). Epoch 20–30 —
all 50 reach and park at the green sphere in perfect unison. The "watch
the learning happen" moment.

**Validates:**
- All 50 envs converge to same behavior (max pairwise qpos difference < 0.1)
- ≥40/50 trigger done by epoch 30
- Reward improvement ≥80% from epoch 1
- Faster convergence than CEM (#12)

### #15. ppo ★ — "Actor-Critic: Reaching Arm + PPO"

**The industry standard.** Same arm, same target — PPO (Proximal Policy
Optimization). Adds a value function (critic) for variance reduction and
a clipped surrogate objective for stable updates. The visual result:
faster and smoother convergence than REINFORCE, with less oscillation in
the learning curve.

**Why this matters:** REINFORCE has high variance — the gradient estimate
is noisy because it uses raw returns. PPO reduces variance by subtracting
a learned baseline (value function) and prevents destructive large updates
via clipping. This is why PPO is the default algorithm in robotics RL.

**Architecture (hand-coded, no autograd):**
- **Actor:** same Gaussian linear policy as #14, 10 params
- **Critic:** linear value function `V(s) = w_v · obs + b_v`, 5 params
- **Total:** 15 params (still analytically tractable)

**Key PPO additions over REINFORCE:**
```
1. GAE (Generalized Advantage Estimation):
   Aₜ = Σ (γλ)^k δₜ₊ₖ   where δₜ = rₜ + γV(sₜ₊₁) - V(sₜ)

2. Clipped surrogate objective:
   L = min(rₜ·Aₜ, clip(rₜ, 1-ε, 1+ε)·Aₜ)
   where rₜ = π_new(a|s) / π_old(a|s)

3. Value function loss:
   L_v = (V(s) - R_target)²
```

~150 lines of gradient math. Still no autograd — the linear policy and
linear critic both have analytically computable gradients.

**Visual story:** Converges noticeably faster than REINFORCE. By epoch
10–15, all 50 arms reach smoothly. The learning curve (shown in HUD) is
smoother — less oscillation between epochs.

**Validates:**
- ≥40/50 trigger done by epoch 20 (faster than REINFORCE's epoch 30)
- Learning curve monotonically improves (no large regressions)
- Value function loss decreases over training
- Clipping activates in early epochs (prevents destructive updates)

### #13. terminal-obs — "Terminal Observations for Value Bootstrapping"

4 pendulums. Done at |qpos| > 2.0, truncated at t > 2.0. When an env
resets, HUD highlights terminal obs vs initial obs for 1 second. Shows
"Bootstrap?" column (done=No, truncated=Yes).

**MJCF:** Same as #2.

**Validates:**
- On done: terminal_obs qpos beyond threshold, observations qpos near zero
- On truncated: terminal_obs also populated
- Non-reset envs have terminal_observations == None
- dones and truncateds are mutually exclusive (done takes precedence)
- After auto-reset, env continues stepping normally

## Phase 5: 6-DOF Architecture Ladder

The 2-DOF algorithm ladder (Phase 4) proved all 5 algorithms work with
linear policies. Phase 5 escalates: harder task (6-DOF), deeper networks
(MLP, autograd), and the algorithms that benefit most from capacity.

**Key insight from competition tests:** CEM dominates on 2-DOF because the
problem is trivially solvable by sampling 10 parameters. On 6-DOF, the
story reverses — gradient methods with deeper networks overtake CEM.
Phase 5 makes this visible.

**Physics:** 3-segment planar arm (seg1→seg2→seg3), 6 hinge joints
(j1–j6 alternating pitch/yaw), 6 motors with decreasing gear ratios
(10→8→6→5→4→3). Fingertip site at end of seg3. Gravity enabled,
contacts disabled. dt=0.002, RK4. Defined in `TaskConfig::reaching_6dof()`.

**Shared lib:** Extend `shared/src/lib.rs` with `setup_reaching_6dof_arms()`
for the 3-segment arm geometry.

### #18. stress-test (headless)

6-DOF-specific integration checks. The crate already has unit tests for
`reaching_6dof()` (dims, build, step), but this covers visual-example
prerequisites:

1. **6-DOF VecEnv parity** — 50-env VecEnv matches sequential stepping
2. **Obs scaling sanity** — obs_scale [1/π×6, 0.1×6] keeps values in
   reasonable range after 100 steps
3. **Done condition reachable** — starting from target_joints, fingertip
   is within 5cm of target (done fires)
4. **Truncated timing** — episode truncates at 5.0s
5. **Reward gradient** — reward improves as qpos approaches target_joints
6. **Auto-reset terminal_obs** — terminal observations populated correctly
7. **6-DOF linear policy** — 72 params (6×12), forward produces valid
   actions in [-1,1] (tanh)
8. **6-DOF MLP policy** — hidden=32, 614 params, forward produces valid
   actions
9. **6-DOF autograd policy** — 2-layer [64,64] ReLU, forward matches
   expected param count, gradient finite after backward
10. **No NaN after 500 steps** — endurance with random actions

### #19. cem-linear — "6-DOF Reaching: CEM + Linear Policy"

**The familiar algorithm on a harder task.** 50 three-segment arms learn to
reach a target via CEM. Same CEM that was king on 2-DOF — but 6-DOF
has 12-dim obs and 6-dim action, so the linear policy now has 78 params
(6×12 weights + 6 biases). CEM needs more envs or more generations to
search this space.

**Task:** `TaskConfig::reaching_6dof()`. Target joints:
[0.5, 0.2, -0.8, 0.1, 0.5, -0.1]. Reward: negative squared joint-space
error. Done: fingertip within 5cm + vel < 1.0. Truncated: t > 5.0s.
Sub-steps: 5.

**Policy:** `LinearPolicy` — `action = tanh(W · obs_scaled + b)`, 78 params.
Obs scaled by [1/π×6, 0.1×6].

**CEM config:** 50 envs, elite_fraction ~0.2, noise_std ~1.0 (higher than
2-DOF because more params need wider search). 40 generations.

**Visual story:** Gen 1 — 3-segment arms flail in all directions (pitch and
yaw). Gen 10–20 — some arms find partial solutions (seg1 points toward
target but seg2/seg3 don't coordinate). Gen 30–40 — CEM finds decent
solutions but fewer arms converge than 2-DOF CEM did. The visual contrast
with 2-DOF auto-reset (#12) is immediate: harder problem, less convergence.

**HUD:** Generation, phase, reached count, best reward, sigma, per-env
scoreboard (same layout as #12 but adapted for 6-DOF metrics).

**Validates:**
- Some arms reach target (≥5/50 trigger done by gen 40)
- Reward improves from gen 1 (≥30% improvement)
- Sigma decreases over training
- Visual: arms coordinate across 3 segments, not just seg1

### #20. ppo-mlp — "6-DOF Reaching: PPO + MLP"

**Gradient methods shine with capacity.** Same 6-DOF task as #19 — but
replaces CEM with PPO and upgrades from linear to MLP (1 hidden layer,
32 units). The MLP has 614 params (much more than CEM's 78 linear params),
but PPO uses gradients to navigate this space efficiently.

**Why this matters:** Linear policies can't represent nonlinear value
landscapes well. MLP adds a ReLU/tanh hidden layer that lets the policy
learn joint coordination patterns CEM's linear policy can't capture. The
competition tests (`hypothesis_mlp_beats_linear`) confirm MLP PPO
outperforms linear PPO on 6-DOF.

**Policy:** `MlpPolicy` — hidden=32, tanh activation. 614 params.
**Critic:** `MlpValue` — hidden=32, same architecture. ~200 params.
**Optimizer:** Adam, lr ~3e-4.

**PPO config:** 50 envs, clip_eps=0.2, k_passes=2–4, gamma=0.99,
gae_lambda=0.95, sigma annealing. 40 epochs.

**Visual story:** Epoch 1 — random flailing (like CEM gen 1). Epoch 10 —
all 50 arms moving in unison toward target (shared policy, unlike CEM's
per-env perturbations). Epoch 20–30 — coordinated 3-segment reaching,
most arms park near target. The contrast with #19: more convergence,
smoother motion, all arms in sync.

**HUD:** Epoch, phase, reached count, mean reward, sigma, value loss,
clip fraction, reward curve.

**Validates:**
- ≥20/50 trigger done by epoch 30
- Reward improvement ≥60% from epoch 1
- Value loss decreases over training
- Visual: all arms move in unison (shared policy)

### #21. sac-autograd — "6-DOF Reaching: SAC + Autograd 2-Layer"

**Deep network + entropy at the frontier.** Same 6-DOF task — SAC with
autograd-backed 2-hidden-layer networks ([64,64] ReLU, Xavier init).
This is the most capable architecture in the crate: automatic
differentiation through arbitrary-depth networks, stochastic policy with
learned log-std, entropy-regularized exploration, and twin Q-networks.

**Why this matters:** SAC requires `StochasticPolicy` for reparameterized
sampling. Only `AutogradStochasticPolicy` provides this with MLP capacity
(there's no `MlpStochasticPolicy`). This example is the visual proof that
the autograd system — tape, backward, Xavier init, ReLU layers — works
end-to-end in a real learning loop.

**Policy:** `AutogradStochasticPolicy` — 2 hidden layers [64,64], ReLU,
Xavier init. Stochastic: learned log_std per action dimension.
**Critics:** `AutogradQ` × 2 (twin Q-networks), same architecture.
**Optimizer:** Adam, lr ~3e-4. Separate optimizers for actor and critics.
**Alpha:** auto-tuned, target_entropy = -act_dim.

**SAC config:** 50 envs, gamma=0.99, tau=0.005, batch_size=256,
buffer_capacity ~50k, warmup_steps ~1000. 50 epochs.

**Visual story:** Steps 1–1000 (warmup) — random exploration fills replay
buffer. Epoch 5–10 — arms begin coordinating, entropy high (diverse
exploration). Epoch 20–30 — arms converge smoothly, alpha decreases as
policy becomes more confident. Epoch 40–50 — tight reaching, most arms
at target. The contrast with #20: off-policy efficiency means more
sample reuse, entropy prevents premature convergence.

**HUD:** Step count, epoch, phase (warmup/running/paused), replay buffer
fill, mean Q1, alpha, entropy, reached count, reward curve.

**Validates:**
- ≥15/50 trigger done by epoch 40
- Reward improvement ≥40% (early to late episodes)
- Alpha adapted (|final - initial| > 0.001)
- Replay buffer fills to warmup threshold
- Visual: arms explore diversely early, converge late

## Phase 6: Obstacle Avoidance

The hardest learning task. A ghost sphere sits between the arm's rest
configuration and the target. The agent must curve around it. The reward
includes a distance penalty:
`r = -dist(fingertip, target) - 10 * max(0, 0.12 - dist(fingertip, obstacle))`

This makes the reward landscape non-convex — the shortest path to the
target goes through the obstacle, so the arm must find a longer path
around it.

**Physics:** Same 3-segment arm as Phase 5. Obstacle at (0.730, 0.046, 0.030),
radius 0.06. Target at (0.681, 0.154, 0.101). Defined in
`TaskConfig::obstacle_reaching_6dof()`.

**Observation:** 21-dim: qpos(6) + qvel(6) + fingertip_xpos(3) +
obstacle_xpos(3) + target_xpos(3). The agent sees where the obstacle
and target are.

### #22. stress-test (headless)

Obstacle-specific integration checks beyond the crate's 6 unit tests:

1. **Penalty fires near obstacle** — at rest (qpos=0), fingertip ≈ 0.058m
   from obstacle center, well within r_safe=0.12, penalty > 0
2. **Penalty zero when far** — at target config, fingertip far from obstacle,
   penalty == 0.0
3. **Reward gradient toward target** — moving qpos toward target_joints
   improves reward (when not near obstacle)
4. **Obstacle position in obs** — obs[15..18] matches body 4 xpos
5. **Target position in obs** — obs[18..21] matches site 0 xpos
6. **50-env VecEnv parity** — batched stepping matches sequential
7. **500-step endurance** — no NaN with random actions near obstacle
8. **Done reachable** — from target_joints, fingertip within 5cm, done fires

### #23. sac-autograd — "Obstacle Avoidance: SAC + Autograd 2-Layer"

**The capstone learning example.** 50 three-segment arms learn to reach
the green target while avoiding the red obstacle. SAC + autograd 2-layer
([64,64] ReLU) — the same architecture as #21, but the obstacle penalty
forces the arm to find a non-trivial path.

**Visual:** Same 3-segment arm layout as Phase 5 examples. Add:
- Red semi-transparent sphere at obstacle position (r=0.06)
- Green sphere at target position
- Arms that clip the obstacle's safe radius (r=0.12) should visually
  show the proximity (e.g., segment near obstacle turns amber/red)

**SAC config:** Same as #21 but with the obstacle task. May need more
epochs (50–80) due to harder exploration.

**Visual story:** Warmup — random exploration, many arms clip the obstacle.
Epoch 10–20 — arms begin routing around the obstacle, some from above,
some below. Epoch 30–50 — most arms find clean paths that curve around
the obstacle to the target. The penalty visualization shows fewer
near-misses as training progresses.

**HUD:** Same as #21, plus:
- Obstacle penalty (mean penalty across envs this epoch)
- Near-miss count (envs where fingertip entered r_safe)

**Validates:**
- ≥10/50 trigger done by epoch 50
- Reward improvement ≥30% (early to late)
- Mean obstacle penalty decreases over training
- Visual: arms curve around obstacle, not through it

## Phase 7: Persistence Depth

The existing `train-then-replay` example (#P1) demonstrates saving and
loading a `PolicyArtifact` — policy weights only. But the crate has full
`TrainingCheckpoint` support: policy, critics, optimizer momentum (m, v, t),
algorithm-specific state (noise_std, sigma, log_alpha), and best-epoch
tracking. None of that has visual proof.

### #24. checkpoint-resume — "Checkpoint Resume: Pause, Save, Continue"

**Full checkpoint round-trip.** Train a CEM on 2-DOF reaching for 15 epochs
→ save a `TrainingCheckpoint` to disk → reload the checkpoint → continue
training for 15 more epochs. The learning curve should be seamless across
the boundary — no regression, no re-exploration.

**Why CEM on 2-DOF:** The concept is persistence, not the learning problem.
CEM on 2-DOF converges visibly in 15–20 epochs, making the resume effect
obvious. A harder task would obscure the signal.

**Two-phase visual:**
- **Phase A (epochs 1–15):** Live CEM training, 50 reaching arms. HUD shows
  epoch, reward, sigma, best-epoch tracking. At epoch 15, training pauses.
  HUD shows "Saving checkpoint..." then the file path.
- **Phase B (epochs 16–30):** Checkpoint loaded. Training resumes. HUD shows
  "Resumed from checkpoint (epoch 15)" and the learning curve continues
  from where it left off. Best-epoch tracker carries over — if the best
  epoch was epoch 12, the HUD shows "Best: epoch 12 (reward: -X.XX)" and
  only updates if a later epoch beats it.

**Best-policy tracking in HUD:** Every frame shows:
```
Best epoch: 12  reward: -0.83  (params saved)
```
This persists across the checkpoint boundary. If epoch 22 beats epoch 12,
it updates live. This is the visual proof that `BestTracker` works.

**Checkpoint contents saved:**
- `algorithm_name: "CEM"`
- `policy_artifact` (current policy weights)
- `algorithm_state: { "noise_std": ... }`
- `best_params`, `best_reward`, `best_epoch`

**Validates:**
- Reward at epoch 16 ≥ reward at epoch 15 (no regression after resume)
- noise_std at epoch 16 matches noise_std at epoch 15 (state preserved)
- Best epoch/reward carries across checkpoint boundary
- Final reward (epoch 30) ≥ reward at checkpoint (epoch 15)
- Checkpoint file written and read successfully

## Phase 8: Competition Dashboard

The crate has a `Competition` runner with 13 integration tests validating
algorithm ordering hypotheses. This example makes it visual: train all 5
algorithms headlessly on the same task, then replay each algorithm's best
policy.

### #25. dashboard — "Competition: 5 Algorithms on 6-DOF"

**Multi-algorithm comparison.** Headless CEM/REINFORCE/PPO/TD3/SAC training
on `reaching_6dof()`, then replay each algorithm's `best_artifact()` in a
row of 5 arms.

**Training phase (headless):** Use `Competition::new(50, Epochs(30), 42)`.
All 5 algorithms get autograd 2-layer [64,64] networks (level 2). Training
runs headlessly at startup — progress bar or HUD showing "Training CEM...
REINFORCE... PPO... TD3... SAC..." with wall times.

**Replay phase (visual):** 5 arms side by side, each executing the best
policy from one algorithm. Arms color-coded and labeled:

| Position | Algorithm | Color |
|----------|-----------|-------|
| 1 (leftmost) | CEM | Blue |
| 2 | REINFORCE | Cyan |
| 3 | PPO | Green |
| 4 | TD3 | Orange |
| 5 (rightmost) | SAC | Red |

**HUD (per arm):**
```
CEM         REINFORCE   PPO         TD3         SAC
-0.83       -12.4       -3.1        -8.7        -5.2
ep 22/30    ep 18/30    ep 25/30    ep 14/30    ep 20/30
3.2s        4.1s        5.8s        6.2s        7.1s
```
Shows: algorithm name, best reward, best epoch, training wall time.

**Visual story:** Viewers immediately see which algorithms learned to reach
(arms at target) vs which didn't (arms stuck or flailing). The ranking
matches the competition test findings — likely CEM or SAC at the top for
this configuration.

**Validates:**
- All 5 algorithms complete training without panic
- At least 2 algorithms achieve meaningful reward improvement (≥30%)
- Best policies replay without error
- Arms visually differentiated (different colors, labels)

## Implementation Order

Stress-test first in each phase — establishes correctness before investing
in visual boilerplate. If the API has issues, we find them cheaply. Then
visual examples build on a confirmed-working foundation.

Build one at a time. Review each (numbers pass + visuals pass) before the
next:

### Phase 1: Spaces — COMPLETE
1. **stress-test** — headless, 9 integration checks — **DONE**
2. obs-extract — **DONE**
3. act-inject — **DONE**
4. obs-rich — **DONE**
5. act-clamping — **DONE**

### Phase 2: SimEnv — COMPLETE
6. **stress-test** — **DONE**
7. episode-loop — **DONE**
8. sub-stepping — **DONE**
9. on-reset — **DONE**

### Phase 3: VecEnv — DONE except #13
10. **stress-test** — **DONE**
11. parallel-step — **DONE**
12. auto-reset — ★ CEM — **DONE**
13. terminal-obs — to build

### Phase 4: 2-DOF Algorithm Ladder — COMPLETE
14. reinforce — **DONE**
15. ppo — **DONE** (README missing)
16. td3 — **DONE** (README missing)
17. sac — **DONE** (README missing)

### Housekeeping
- terminal-obs (#13)
- PPO/TD3/SAC READMEs

### Phase 5: 6-DOF Architecture Ladder
18. **stress-test** — headless, 10 integration checks
19. cem-linear — ★ 6-DOF + CEM + linear (task complexity)
20. ppo-mlp — ★ 6-DOF + PPO + MLP (architecture upgrade)
21. sac-autograd — ★ 6-DOF + SAC + autograd 2-layer (deep network)

### Phase 6: Obstacle Avoidance
22. **stress-test** — headless, 8 obstacle checks
23. sac-autograd — ★ 50 arms + obstacle avoidance

### Phase 7: Persistence Depth
24. checkpoint-resume — full checkpoint save/resume + best-policy tracking

### Phase 8: Competition Dashboard
25. dashboard — 5 algorithms on 6-DOF, best policies replayed side by side

## READMEs

Every example gets a museum-plaque README.md.

### Visual examples

```markdown
# [Title]

[One sentence: what concept this demonstrates.]

## What you see
- Description of each visual element and what it represents

## Expected behavior
- What the viewer should observe happening over time
- Numerical values visible in the HUD and what they mean

## Validation
| Check | Expected | Tolerance |
|-------|----------|-----------|
| ...   | ...      | ...       |

## Run
cargo run -p example-ml-{name} --release
```

### Stress tests

```markdown
# [Group] Stress Test

Headless validation of [group] edge cases and correctness invariants.

## What it tests
- Numbered list of every check with a one-line description of what
  it verifies and why that matters

## Expected output
- All checks PASS
- Example console output snippet

## Run
cargo run -p example-ml-{group}-stress-test --release
```

## Key References

- **Pattern A template:** `examples/fundamentals/sim-cpu/actuators/motor/src/main.rs`
- **Pattern C template:** `examples/fundamentals/sim-cpu/batch-sim/reset-subset/src/main.rs`
- **sim-bevy helpers:** `sim/L1/bevy/src/examples.rs`
- **Multi-scene:** `sim/L1/bevy/src/multi_scene.rs`
- **Model/Data:** `sim/L1/bevy/src/model_data.rs`
- **6-DOF task:** `sim/L0/ml-bridge/src/task.rs` — `reaching_6dof()`, `obstacle_reaching_6dof()`
- **Competition runner:** `sim/L0/ml-bridge/src/competition.rs`
- **Competition tests:** `sim/L0/ml-bridge/tests/competition.rs` (13 hypothesis tests)
- **Artifact/checkpoint:** `sim/L0/ml-bridge/src/artifact.rs`
- **Best-policy tracker:** `sim/L0/ml-bridge/src/best_tracker.rs`
- **Autograd:** `sim/L0/ml-bridge/src/autograd.rs`, `autograd_policy.rs`, `autograd_value.rs`
- **MLP networks:** `sim/L0/ml-bridge/src/mlp.rs`

## Verification

Each example verified by:
1. `cargo run -p example-ml-{name} --release` — runs without panic
2. Validation report prints all PASS at report time
3. Visual inspection: geometry renders, HUD updates, behavior matches README
4. `cargo clippy -p example-ml-{name} -- -D warnings` — clean
