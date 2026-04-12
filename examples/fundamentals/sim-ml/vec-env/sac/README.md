# SAC — Reaching Arm + Soft Actor-Critic

Fifty two-link arms learn to reach a green target using SAC (Soft
Actor-Critic). Off-policy like TD3, but with a stochastic policy that
learns its own exploration noise — per-action log-std parameters are
trained alongside the policy weights. An entropy temperature α is
auto-tuned to balance exploitation and exploration. The visual story:
random warmup, then arms coordinate with visible diversity early (high
entropy) that narrows as α decreases.

See also: [TD3](../td3/) — off-policy with fixed exploration noise.
[PPO](../ppo/) — on-policy actor-critic.

## What you see

- **50 arms in a row**, identical two-link planar arms (steel-blue upper
  arm, warm-orange forearm). All share one stochastic policy
- **Green spheres** at target position (0.4, 0, 0.3)
- **Warmup phase** — arms execute random actions for the first 50 steps
  per env (2500 total) to fill the replay buffer. HUD shows "Warmup"
- **Early training (high entropy)** — arms move toward the target but
  with visible action diversity. Each arm follows the same mean policy
  but the learned noise (σ) is still high, so trajectories vary
- **Late training (low entropy)** — α has decreased, σ has tightened.
  Arms converge with less variation, steadily improving reward
- **PAUSED** — 1-second freeze between display epochs for visual clarity
- **HUD** (bottom-left) — step count, display epoch, phase, replay buffer
  fill, mean Q1 value, actor gradient norm, **alpha** and **entropy**
  (unique to SAC), reached count, and reward curve

## How it works

### Physics

Identical to CEM, REINFORCE, PPO, and TD3 — same MJCF, gear ratios,
damping, target.

| Parameter | Value |
|-----------|-------|
| Upper arm | L=0.5 m, mass=0.5 kg, damping=2.0 |
| Forearm | L=0.4 m, mass=0.3 kg, damping=1.0 |
| Shoulder motor | gear=10, ctrl range [-1, 1] |
| Elbow motor | gear=5, ctrl range [-1, 1] |
| Target | (0.4, 0, 0.3) — IK solution: qpos=(-0.242, 1.982) |
| Integrator | RK4, dt=0.002 s, no contacts |

### Policy and critics

**Stochastic actor:** Linear mean + learned log-std —
**a = tanh(W * obs_scaled + b) + exp(log_std) * ε**, where ε ~ N(0, I).
12 parameters: 10 for the mean (same as all other examples) + 2 for
per-action log_std.

This is the key difference from TD3: TD3 uses fixed Gaussian noise
(σ=0.3) that doesn't adapt. SAC learns σ per action dimension — the
policy discovers which actions need more or less exploration.

**Twin Q-networks:** Same as TD3 — two independent linear Q-functions,
7 parameters each.

**Target networks:** Same as TD3 — Polyak-averaged copies of both critics.

| Component | Params | Role |
|-----------|--------|------|
| Stochastic actor | 12 | Mean policy + learned exploration noise |
| Q1, Q2 | 7 each | Twin critics |
| Target Q1, Q2 | 7 each | Smoothed critics for TD targets |
| log_alpha | 1 | Entropy temperature (auto-tuned) |

### SAC algorithm

Four innovations over TD3:

**1. Stochastic policy with reparameterization trick:**
```
a = μ(s) + exp(log_std) * ε      where ε ~ N(0, I)
```
This allows differentiation through the sampling process — gradients
flow from Q through the sampled action back to the policy parameters.

**2. Entropy-regularized Q-targets:**
```
y = r + γ (min(Q1'(s', a'), Q2'(s', a')) - α log π(a'|s'))
```
The `-α log π` bonus rewards exploration: actions with high entropy
(spread-out probability) get higher Q-targets. This prevents the
premature convergence that plagues TD3 on this task.

**3. Policy gradient with entropy bonus:**
```
∇θ J = ∇θ (Q(s, a) - α log π(a|s))
```
The policy maximizes expected Q-value while maintaining entropy. α
controls the tradeoff.

**4. Alpha auto-tuning:**
```
∇α = -α (log π(a|s) + target_entropy)
```
When entropy drops below the target, α increases (more exploration
pressure). When entropy exceeds the target, α decreases (more
exploitation). Target entropy = -act_dim = -2 (heuristic).

| Parameter | Value | Why |
|-----------|-------|-----|
| Envs | 50 | Same budget |
| Max steps | 6000 | ~120 steps/env |
| Warmup | 50 steps/env | 2500 transitions before learning |
| Batch size | 64 | Mini-batch from replay buffer |
| Buffer capacity | 100k | Stores all experience |
| gamma | 0.95 | Same as TD3 |
| tau | 0.005 | Same as TD3 |
| lr_actor | 5e-5 | Same as TD3 |
| lr_critic | 5e-3 | Same as TD3 |
| lr_alpha | 5e-3 | Fast alpha adaptation |
| init_log_std | -0.5 | σ ≈ 0.6, moderate initial noise |
| init_log_alpha | -4.6 | α ≈ 0.01, matching reward scale |
| reward_scale | 0.01 | Prevents Q-value explosion |

### TD3 vs SAC on this task

TD3 achieves ~30% improvement then plateaus — the deterministic policy
with fixed noise can't explore effectively once it's near a local optimum.
SAC's entropy regularization prevents this: the `-α log π` bonus keeps
the policy exploring even as it improves, leading to ~50% steady
improvement. The alpha adaptation in the HUD makes this visible — α
starts low, adjusts to maintain the target entropy, and the learning
curve stays positive.

On this 2-DOF task with linear policies, the advantage is modest. On
harder tasks (6-DOF, obstacle avoidance), entropy regularization becomes
essential — see Phase 5 and Phase 6 examples.

## Validation (at epoch 15)

| Check | Expected |
|-------|----------|
| Reward improved | > 10% (late episodes vs early) |
| Replay buffer filled | >= warmup_steps * num_envs |
| Alpha adapted | abs(final_alpha - init_alpha) > 0.001 |

## Run

```
cargo run -p example-ml-vec-env-sac --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
