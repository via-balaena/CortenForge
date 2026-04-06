# REINFORCE — Reaching Arm + Policy Gradient

Fifty two-link arms learn to reach a green target using REINFORCE (vanilla
policy gradient). Unlike CEM where each env runs a different perturbation,
all 50 envs share ONE policy — exploration comes from Gaussian noise on
actions. The visual difference is dramatic: all arms move in unison,
improving together each epoch.

See also: [Auto-Reset (CEM)](../auto-reset/) — same arm, same target, but
sampling-based optimization instead of gradients.

## What you see

- **50 arms in a row**, identical two-link planar arms (steel-blue upper
  arm, warm-orange forearm). All share one policy and one VecEnv
- **Green spheres** at target position (0.4, 0, 0.3)
- **Epoch 1: chaos in unison** — random policy outputs near-zero mean
  actions, all arms swing similarly under gravity with noise variation
- **Epochs 5-10: directional learning** — all 50 arms swing toward the
  target together, same mean trajectory with decreasing noise
- **Epochs 20-30: near-convergence** — arms consistently reach the target
  region. Reward improves 90%+ from epoch 1. The arms move as one
- **UPDATING pause** — 1.5-second freeze between epochs for the gradient
  computation (Adam update, advantage normalization)
- **HUD** (bottom-left) — epoch counter, phase, reach count, mean reward,
  sigma, gradient norm, and a compact reward curve

## How it works

### Physics

Identical to the CEM example — same MJCF, gear ratios, damping, target.

| Parameter | Value |
|-----------|-------|
| Upper arm | L=0.5 m, mass=0.5 kg, damping=2.0 |
| Forearm | L=0.4 m, mass=0.3 kg, damping=1.0 |
| Shoulder motor | gear=10, ctrl range [-1, 1] |
| Elbow motor | gear=5, ctrl range [-1, 1] |
| Target | (0.4, 0, 0.3) — IK solution: qpos=(-0.242, 1.982) |
| Integrator | RK4, dt=0.002 s, no contacts |

### Policy

Same linear structure as CEM:
**mean: mu(s) = tanh(W * obs_scaled + b)** — 10 parameters

**Stochastic: a = mu(s) + sigma * N(0, I)**

The key difference: CEM perturbs the *parameters* (each env gets different
W, b). REINFORCE perturbs the *actions* (same W, b for all envs, different
noise per step). This is why all arms look similar — they share one policy.

### REINFORCE algorithm

Each epoch: collect 50 full-episode trajectories, compute the policy
gradient, update parameters with Adam.

**Gradient (hand-coded, no autograd):**
```
d/dW log pi(a|s) = (a - mu) / sigma^2 * (1 - mu^2) * s_scaled
d/db log pi(a|s) = (a - mu) / sigma^2 * (1 - mu^2)
```

| Parameter | Value | Why |
|-----------|-------|-----|
| Envs | 50 | Same budget as CEM |
| Epochs | 30 | Same budget as CEM generations |
| lr | 0.05 | Adam handles scale; this controls step magnitude |
| gamma | 0.99 | Covers ~300-step episodes |
| sigma_init | 0.5 | Moderate initial exploration |
| sigma_decay | 0.90 | Anneals to near-deterministic |
| Optimizer | Adam | Essential — vanilla SGD oscillates on noisy PG |
| Advantages | Normalized | Zero-mean, unit-variance for stable gradients |

### CEM vs REINFORCE on this task

CEM achieves 18-23 done triggers per generation (arms reaching within 5cm).
REINFORCE achieves 90%+ reward improvement but few done triggers. This
difference is inherent:

- **CEM** explores globally in parameter space — some perturbations land
  near the optimal policy by chance, and elite selection jumps there
- **REINFORCE** takes local gradient steps — gets all arms pointed at the
  target but can't achieve the precision needed for the 5cm done condition

This motivates PPO (example #15), which adds a value function baseline to
reduce gradient variance, enabling precise convergence.

## Validation (at epoch 25)

| Check | Expected |
|-------|----------|
| Reward improvement | >= 80% from epoch 1 |
| Policy learned | param norm > 0.5 |
| Sigma decreased | below sigma_init * 0.5 |

## Run

```
cargo run -p example-ml-vec-env-reinforce --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
