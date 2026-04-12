# PPO — Reaching Arm + Actor-Critic

Fifty two-link arms learn to reach a green target using PPO (Proximal Policy
Optimization). Builds on REINFORCE: same arm, same VecEnv, same linear
policy — adds a learned value function (critic) for lower-variance advantage
estimates and a clipped surrogate objective that prevents destructive updates.
The visual result: smoother convergence, less oscillation, and arms that
actually park at the target.

See also: [REINFORCE](../reinforce/) — same arm, policy gradient without
a critic. [CEM](../auto-reset/) — sampling-based, no gradients at all.

## What you see

- **50 arms in a row**, identical two-link planar arms (steel-blue upper
  arm, warm-orange forearm). All share one policy and one VecEnv
- **Green spheres** at target position (0.4, 0, 0.3)
- **Epoch 1: random flailing** — near-zero mean actions, arms swing
  under gravity with action noise
- **Epochs 5–10: coordinated swing** — all 50 arms move in unison toward
  the target (shared policy), value function learning visible in HUD as
  value loss decreases
- **Epochs 15–25: precision reaching** — arms park near the target with
  minimal oscillation. The learning curve is smoother than REINFORCE
  because GAE reduces variance
- **UPDATING pause** — 1.5-second freeze between epochs for the PPO update
  (K optimization passes over the rollout data)
- **HUD** (bottom-left) — epoch counter, phase, reached/done counts, mean
  reward, sigma, value loss, clip fraction, actor gradient norm, and a
  compact reward curve showing milestones

## How it works

### Physics

Identical to CEM and REINFORCE — same MJCF, gear ratios, damping, target.

| Parameter | Value |
|-----------|-------|
| Upper arm | L=0.5 m, mass=0.5 kg, damping=2.0 |
| Forearm | L=0.4 m, mass=0.3 kg, damping=1.0 |
| Shoulder motor | gear=10, ctrl range [-1, 1] |
| Elbow motor | gear=5, ctrl range [-1, 1] |
| Target | (0.4, 0, 0.3) — IK solution: qpos=(-0.242, 1.982) |
| Integrator | RK4, dt=0.002 s, no contacts |

### Policy and critic

Same linear structure as REINFORCE:
**Actor: mu(s) = tanh(W * obs_scaled + b)** — 10 parameters
**Stochastic: a = mu(s) + sigma * N(0, I)**

PPO adds:
**Critic: V(s) = w_v * obs_scaled + b_v** — 5 parameters (4 weights + 1 bias)

| Component | Params | Role |
|-----------|--------|------|
| Actor (W, b) | 10 | Maps observations to mean actions |
| Critic (w_v, b_v) | 5 | Estimates state value for advantage computation |
| Total | 15 | Still analytically tractable |

### PPO algorithm

PPO improves on REINFORCE with three additions:

**1. Generalized Advantage Estimation (GAE):**
```
δₜ = rₜ + γ V(sₜ₊₁) - V(sₜ)           (TD residual)
Aₜ = Σₖ (γλ)ᵏ δₜ₊ₖ                     (smoothed advantage)
```
GAE interpolates between high-bias/low-variance (λ=0, pure TD) and
low-bias/high-variance (λ=1, Monte Carlo). λ=0.95 is the standard.

**2. Clipped surrogate objective:**
```
rₜ = π_new(a|s) / π_old(a|s)            (importance ratio)
L = min(rₜ Aₜ, clip(rₜ, 1-ε, 1+ε) Aₜ)  (pessimistic bound)
```
When the new policy diverges too far from the old (ratio outside
[1-ε, 1+ε]), the gradient is clipped. This prevents catastrophically
large updates that REINFORCE is vulnerable to.

**3. Value function loss:**
```
L_v = (V(s) - target)²    where target = Aₜ + V_old(sₜ)
```

| Parameter | Value | Why |
|-----------|-------|-----|
| Envs | 50 | Same budget as CEM/REINFORCE |
| Epochs | 30 | Same budget |
| lr_actor | 0.025 | Adam with gradient clipping |
| lr_critic | 0.03 | Slightly faster than actor |
| gamma | 0.99 | Covers ~300-step episodes |
| gae_lambda | 0.95 | Standard bias-variance tradeoff |
| clip_eps | 0.2 | Standard PPO clipping range |
| K passes | 2 | Optimization passes per epoch |
| sigma_init | 0.5 | Moderate initial exploration |
| sigma_decay | 0.90 | Anneals to near-deterministic |
| max_grad_norm | 0.5 | Prevents gradient explosions |

### REINFORCE vs PPO on this task

REINFORCE achieves 90%+ reward improvement but the learning curve is noisy —
large epoch-to-epoch variance because raw returns have high variance. PPO
fixes this: the value function baseline (GAE) reduces variance, and clipping
prevents the destructive jumps. PPO converges faster and smoother to the
same level.

## Validation (at epoch 25)

| Check | Expected |
|-------|----------|
| Reward improvement | >= 80% from epoch 1 |
| Value loss decreased | last epoch < first epoch |
| Clipping activated | clip_fraction > 0 in at least one epoch |

## Run

```
cargo run -p example-ml-vec-env-ppo --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
