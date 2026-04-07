# sim-ml-bridge Visual Examples Spec

> **Status**: Draft v3
> **Location**: `examples/fundamentals/sim-ml/`
> **Depends on**: `sim-ml-bridge`, `sim-bevy`, `sim-core`, `sim-mjcf`

## Context

The sim-ml-bridge crate (Phases 1–6 complete, 111 tests) needs visual Bevy
examples following the same "baby steps, one concept per example" pattern as
`examples/fundamentals/sim-cpu/`. The crown jewels should have "watch the
learning happen" quality — a population of arms going from chaos to
coordinated reaching.

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
├── spaces/
│   ├── stress-test/          #1  — headless correctness gate
│   ├── obs-extract/          #2  — see tensor values update live
│   ├── act-inject/           #3  — action drives the pendulum
│   ├── obs-rich/             #4  — 13 extractor types in one HUD
│   └── act-clamping/         #5  — out-of-range actions get clipped
├── sim-env/
│   ├── stress-test/          #6  — headless correctness gate
│   ├── episode-loop/         #7  — Pattern B spike + episode lifecycle
│   ├── sub-stepping/         #8  — side-by-side 1 vs 10 sub-steps
│   └── on-reset/             #9  — different starting angle each episode
└── vec-env/
    ├── stress-test/          #10 — headless correctness gate
    ├── parallel-step/        #11 — 8 pendulums, different actions
    ├── auto-reset/           #12 — ★ 50 reaching arms + CEM (sampling)
    ├── terminal-obs/         #13 — done vs truncated, value bootstrapping
    ├── reinforce/            #14 — ★ 50 reaching arms + REINFORCE (gradient)
    └── ppo/                  #15 — ★ 50 reaching arms + PPO (actor-critic)
```

15 examples total. Package naming: `example-ml-{group}-{name}`.

Examples #12, #14, #15 form a **learning algorithm ladder**: same arm,
same target, same VecEnv — three different optimizers. The visual
progression tells the story of the level 0-1 landscape:

| # | Algorithm | Type | Visual result |
|---|-----------|------|---------------|
| 12 | CEM | Sampling-based | ★ Best performer — arms converge and reach target |
| 14 | REINFORCE | Policy gradient | Arms improve but can't reach precisely |
| 15 | PPO | Actor-critic | Better than REINFORCE, still below CEM |

**Note on ordering:** the original spec predicted gradient methods would
dominate CEM. Phase 3 competition tests (seed 42, 50ep/50env) showed the
**opposite** at level 0-1: CEM (-1.05, 49 dones) >> TD3/SAC (-11) >>
PPO (-3449) >> REINFORCE (-7500). Hand-coded gradients in 614-dim space
are too noisy to outperform gradient-free search on a smooth quadratic
reward. The visual examples should reflect what actually happens — CEM
as the crown jewel, gradient methods as the "why autograd matters"
motivation. See `sim/docs/COMPETITION_TESTS_SPEC.md` for full analysis.

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

## Implementation Order

Stress-test first in each phase — establishes correctness before investing
in visual boilerplate. If the API has issues, we find them cheaply. Then
visual examples build on a confirmed-working foundation.

Build one at a time. Review each (numbers pass + visuals pass) before the
next:

### Phase 1: Spaces
1. **stress-test** — headless, 9 integration checks
2. obs-extract — establishes visual boilerplate for Phase 1
3. act-inject — adds action system to the pattern
4. obs-rich — more complex model (cart-pole), rich HUD
5. act-clamping — focused concept, completes spaces coverage

### Phase 2: SimEnv
6. **stress-test** — headless, 10 integration checks
7. episode-loop — **Pattern B spike** + episode lifecycle
8. sub-stepping — PhysicsScenes for SimEnv side-by-side
9. on-reset — domain randomization

### Phase 3: VecEnv
10. **stress-test** — headless, 11 integration checks
11. parallel-step — introduces Pattern C (VecEnv rendering) — **DONE**
12. auto-reset — ★ reaching arm + CEM (sampling-based) — **DONE**
13. terminal-obs — deep RL concept, HUD-focused

### Phase 4: Learning Algorithm Ladder
14. reinforce — ★ same arm + REINFORCE (gradient-based)
15. ppo — ★ same arm + PPO (actor-critic)

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

## Verification

Each example verified by:
1. `cargo run -p example-ml-{name} --release` — runs without panic
2. Validation report prints all PASS at report time
3. Visual inspection: geometry renders, HUD updates, behavior matches README
4. `cargo clippy -p example-ml-{name} -- -D warnings` — clean
