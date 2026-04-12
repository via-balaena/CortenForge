# TD3 — Reaching Arm + Twin Delayed DDPG

Fifty two-link arms learn to reach a green target using TD3 (Twin Delayed
Deep Deterministic Policy Gradient). This is the first off-policy algorithm
in the ladder: transitions are stored in a replay buffer and reused for
learning, rather than discarded after each epoch. The visual story: a warmup
phase fills the buffer with random experience, then arms gradually
coordinate as the deterministic policy improves.

See also: [PPO](../ppo/) — on-policy actor-critic (uses each transition
once). [SAC](../sac/) — off-policy with entropy regularization.

## What you see

- **50 arms in a row**, identical two-link planar arms (steel-blue upper
  arm, warm-orange forearm). All share one deterministic policy
- **Green spheres** at target position (0.4, 0, 0.3)
- **Warmup phase** — arms execute random actions for the first 50 steps
  per env (2500 total) to fill the replay buffer. HUD shows "Warmup"
  and buffer fill progress
- **Running phase** — after warmup, the policy drives actions with
  Gaussian exploration noise (σ=0.3). Arms begin coordinating, but
  convergence is slower than on-policy methods because the policy
  only updates every 2 critic updates (delayed policy update)
- **PAUSED** — 1-second freeze between display epochs for visual clarity
- **HUD** (bottom-left) — step count, display epoch, phase, replay buffer
  fill, mean Q1 value, actor gradient norm, reached count, and a compact
  reward curve

## How it works

### Physics

Identical to CEM, REINFORCE, and PPO — same MJCF, gear ratios, damping,
target.

| Parameter | Value |
|-----------|-------|
| Upper arm | L=0.5 m, mass=0.5 kg, damping=2.0 |
| Forearm | L=0.4 m, mass=0.3 kg, damping=1.0 |
| Shoulder motor | gear=10, ctrl range [-1, 1] |
| Elbow motor | gear=5, ctrl range [-1, 1] |
| Target | (0.4, 0, 0.3) — IK solution: qpos=(-0.242, 1.982) |
| Integrator | RK4, dt=0.002 s, no contacts |

### Policy and critics

**Actor:** Deterministic linear policy — **a = tanh(W * obs_scaled + b)**,
10 parameters. No action noise during evaluation; exploration noise
(σ=0.3) added during data collection only.

**Twin Q-networks:** Two independent linear Q-functions:
**Q(s, a) = w_q * [obs_scaled; a] + b_q** — 7 parameters each (6 weights
from 4 obs + 2 action dims, plus 1 bias).

**Target networks:** Separate copies of actor and both critics, updated
via Polyak averaging (τ=0.005) after each actor update.

| Component | Params | Role |
|-----------|--------|------|
| Actor | 10 | Deterministic policy |
| Target actor | 10 | Smoothed policy for target computation |
| Q1, Q2 | 7 each | Twin critics for overestimation reduction |
| Target Q1, Q2 | 7 each | Smoothed critics for TD targets |

### TD3 algorithm

Three innovations over DDPG:

**1. Twin Q-networks (clipped double Q-learning):**
```
y = r + γ min(Q1'(s', a'), Q2'(s', a'))
```
Taking the min of two Q estimates reduces overestimation bias — a known
failure mode of single-critic methods.

**2. Delayed policy update:**
The actor updates every `POLICY_DELAY` (2) critic updates. This lets the
Q-function stabilize before the policy chases it, preventing oscillation.

**3. Target policy smoothing:**
```
a' = clip(μ'(s') + clip(ε, -c, c), -1, 1)    where ε ~ N(0, σ_target)
```
Noise on target actions smooths the Q-function, preventing it from
exploiting narrow peaks.

| Parameter | Value | Why |
|-----------|-------|-----|
| Envs | 50 | Same budget |
| Max steps | 6000 | ~120 steps/env (shorter episodes than on-policy) |
| Warmup | 50 steps/env | 2500 total transitions before learning starts |
| Batch size | 64 | Mini-batch from replay buffer |
| Buffer capacity | 100k | Stores all experience for reuse |
| gamma | 0.95 | Shorter horizon (off-policy is less sensitive) |
| tau | 0.005 | Slow Polyak averaging for target stability |
| lr_actor | 5e-5 | Very slow — actor must wait for critics |
| lr_critic | 5e-3 | 100x faster than actor — critics lead |
| exploration_noise | 0.3 | Gaussian noise on actions during collection |
| target_noise | 0.2 | Smoothing noise on target actions |
| noise_clip | 0.5 | Bounds on target smoothing noise |
| policy_delay | 2 | Actor updates every 2nd critic update |
| reward_scale | 0.01 | Prevents Q-value explosion |

### Off-policy vs on-policy on this task

On-policy methods (REINFORCE, PPO) use each transition once and discard it.
TD3 stores every transition and resamples mini-batches — this is more
sample-efficient but introduces distributional shift (learning from old
data). On this small 2-DOF task, the sample-efficiency advantage is modest:
TD3 achieves ~30% improvement then plateaus. The linear Q-function saturates
— it can't represent the nonlinear value landscape accurately enough for
the policy gradient to improve further. This motivates MLP and autograd
architectures in Phase 5.

## Validation (at epoch 15)

| Check | Expected |
|-------|----------|
| Reward improved | > 10% (late episodes vs early) |
| Replay buffer filled | >= warmup_steps * num_envs |
| Policy learned | param norm > 0.1 |

## Run

```
cargo run -p example-ml-vec-env-td3 --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
