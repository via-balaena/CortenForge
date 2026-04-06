# REINFORCE Spec — Reaching Arm + Policy Gradient

> **Status**: Implemented
> **Pattern**: C (VecEnv + PhysicsScenes)
> **Builds on**: auto-reset (same arm, target, VecEnv, MJCF, constants)

## Concept

Same 50 two-link arms, same target, same VecEnv — CEM replaced by REINFORCE.
All 50 envs share ONE policy. Exploration = Gaussian noise on actions.
Visual story: all arms move in unison (same policy), improving each epoch.

## What changes from CEM

| | CEM | REINFORCE |
|---|---|---|
| Per-env policy | Different perturbation | Same shared policy |
| Exploration | Perturbation sampling | Gaussian action noise |
| Credit assignment | Whole-rollout ranking | Per-step gradient |
| Update | Elite mean/std | Adam gradient ascent |
| Visual behavior | Diverse (different policies) | Unison (same policy + noise) |

Everything else is inherited: MJCF, physics, obs/act spaces, done/truncated,
sub_steps=5, OBS_SCALE, VecEnv builder.

## Policy

**Mean:** `mu(s) = tanh(W * s_scaled + b)` — 10 params (W[2x4] + b[2])

**Stochastic:** `a = mu(s) + sigma * eps`, where `eps ~ N(0, I)`

## Gradient

```
d/dtheta log pi(a|s) = (a - mu) / sigma^2 * (1 - mu^2) * d/dtheta z
d/dW[a,o] = (a[a] - mu[a]) / sigma^2 * (1 - mu[a]^2) * s_scaled[o]
d/db[a]   = (a[a] - mu[a]) / sigma^2 * (1 - mu[a]^2)
```

Verified against finite-difference (all 10 params match within 1e-3).

## Training loop

```
For each epoch:
  1. reset_all()
  2. Run 50 envs until all done/truncated, record (obs, action, reward)
  3. Compute discounted returns R_t, normalize advantages (zero-mean, unit-var)
  4. Policy gradient averaged over all samples
  5. Adam update (theta += lr * adam_step)
  6. Anneal sigma *= SIGMA_DECAY
```

## Reward

Joint-space squared error: `-(qpos - target_qpos)^2`. Same as CEM.

**Stress-test finding:** Cartesian L1 distance was also tested but plateaus
at ~0.48m — the L1 gradient gives no directional signal about which joints
to move. Joint-space is better for both CEM and REINFORCE.

## Hyperparameters

| Param | Value | Rationale |
|---|---|---|
| NUM_ENVS | 50 | Same as CEM |
| MAX_EPOCHS | 30 | Same budget as CEM generations |
| gamma | 0.99 | ~300-step episodes |
| lr | 0.05 | Tuned: Adam normalizes, this controls step scale |
| sigma_init | 0.5 | Moderate exploration |
| sigma_min | 0.05 | Near-deterministic at convergence |
| sigma_decay | 0.90 | sigma(30) ~ 0.02 |
| max_grad_norm | 1.0 | Prevent large updates |
| Adam beta1/beta2 | 0.9 / 0.999 | Standard Adam |

## Convergence characteristics

REINFORCE with a 10-param linear policy achieves **90%+ reward improvement**
(from -1870 to ~-170) but triggers **few done conditions** (0-3 per epoch).
This is fundamentally different from CEM's 18-23 reaches:

- **CEM** explores globally in parameter space — elite selection jumps to
  good solutions. No gradient needed.
- **REINFORCE** takes local gradient steps — gets the arm in the right
  direction but can't achieve the 5cm precision needed for done.

This is an honest result that motivates PPO (example #15): actor-critic
reduces REINFORCE's variance, enabling precision convergence.

The visual story works: all 50 arms swing toward the target in unison,
with dramatic reward improvement visible in the HUD curve.

## Validation (at epoch 25)

1. **Reward improvement >= 80%** — ep1 → ep25 improvement
2. **Policy learned** — ||theta|| > 0.5
3. **Sigma decreased** — confirms exploration schedule worked

## Assumptions verified

1. **Gradient correct** — finite-difference match for all 10 params
2. **Convergence** — 90%+ reward improvement in 30 epochs (3/3 seeds)
3. **All envs identical** — mean policy produces bit-exact same trajectory
4. **Sigma schedule** — reaches sigma_min by epoch ~25
5. **Baseline reduces variance** — verified quantitatively
6. **Discounted returns** — verified against hand computation
7. **Cartesian vs joint-space** — joint-space is better for both algorithms
