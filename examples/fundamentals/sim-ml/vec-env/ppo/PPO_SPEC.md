# PPO Spec — Reaching Arm + Actor-Critic

> **Status**: Draft
> **Pattern**: C (VecEnv + PhysicsScenes)
> **Builds on**: reinforce (same arm, target, VecEnv, MJCF, constants)

## Concept

Same 50 two-link arms, same target — REINFORCE replaced by PPO. Adds a
learned value function (critic) that dramatically reduces gradient variance,
plus a clipped surrogate objective that prevents destructive updates. The
visual result: all 50 arms reach and park at the target, fixing REINFORCE's
inability to achieve precision.

## What PPO fixes about REINFORCE

REINFORCE stress-testing revealed three problems:

1. **High gradient variance** — raw returns (-1870 to -170) have huge
   variance even with advantage normalization. The gradient direction
   oscillates epoch to epoch. PPO fix: **GAE** uses a learned value function
   as a per-state baseline, reducing variance from "how good was this
   trajectory" to "how much better/worse than expected was this step."

2. **No per-state baseline** — REINFORCE uses a global mean return as
   baseline. This can't distinguish "this state is inherently bad" from
   "this action was bad." PPO fix: **V(s)** provides a state-dependent
   baseline. The advantage A_t = r_t + γV(s_{t+1}) - V(s_t) measures how
   much better the actual outcome was than expected.

3. **Unstable updates** — Adam helps but the policy can still overshoot
   (seen at lr=0.2 where reward diverged after epoch 15). PPO fix:
   **clipped surrogate** bounds how much the policy can change per update,
   preventing catastrophic forgetting.

## Architecture

**Actor** (same as REINFORCE): `mu(s) = tanh(W * s_scaled + b)` — 10 params

**Critic** (new): `V(s) = w_v · s_scaled + b_v` — 5 params (4 weights + 1 bias)

**Sigma**: annealed on a fixed schedule (same as REINFORCE), not learned.
Learning sigma adds complexity and an extra gradient path. With only 30
epochs, a fixed schedule that worked for REINFORCE is good enough.

**Total**: 15 params (10 actor + 5 critic). Still analytically tractable —
no autograd needed.

The critic is linear in the scaled observations. This is the simplest
possible value function. It can represent V(s) ≈ "how far are the joints
from the target, weighted by importance." For a quadratic reward landscape,
a linear V is a reasonable first-order approximation.

## Gradients (hand-coded)

### Actor gradient (clipped surrogate)

```
ratio_t = pi_new(a_t|s_t) / pi_old(a_t|s_t)

For Gaussian policy, the log-ratio is:
  log(ratio_t) = -0.5 * [(a-mu_new)^2 - (a-mu_old)^2] / sigma^2

Clipped objective per sample:
  L_t = min(ratio_t * A_t, clip(ratio_t, 1-eps, 1+eps) * A_t)

Gradient of L_t w.r.t. theta (actor params):
  If ratio_t * A_t <= clipped_ratio * A_t:
    dL/dtheta = A_t * ratio_t * d/dtheta log pi_new(a_t|s_t)
  Else:
    dL/dtheta = 0  (clipped — no gradient)

d/dtheta log pi_new uses the same formula as REINFORCE:
  d/dW = (a - mu) / sigma^2 * (1 - mu^2) * s_scaled
  d/db = (a - mu) / sigma^2 * (1 - mu^2)
```

### Critic gradient

```
V(s) = w_v · s_scaled + b_v

Loss = (V(s) - R_target)^2

d/dw_v = 2 * (V(s) - R_target) * s_scaled
d/db_v = 2 * (V(s) - R_target)
```

Where R_target = GAE lambda-return: R_t = A_t + V_old(s_t).

### GAE (Generalized Advantage Estimation)

```
delta_t = r_t + gamma * V(s_{t+1}) - V(s_t)
A_t = sum_{k=0}^{T-t} (gamma * lambda)^k * delta_{t+k}

Computed backwards (like discounted returns):
  A_{T-1} = delta_{T-1}
  A_t = delta_t + gamma * lambda * A_{t+1}
```

GAE(lambda=1) = standard discounted returns minus V(s) (= REINFORCE + baseline).
GAE(lambda=0) = one-step TD error (= pure actor-critic, high bias).
GAE(lambda=0.95) = practical sweet spot.

### Terminal state bootstrap (common implementation bug)

At the last step T of a trajectory, delta_T needs V(s_{T+1}):

- **done** (arm reached target): V(s_{T+1}) = 0. The episode truly ended;
  there is no future value.
- **truncated** (timeout): V(s_{T+1}) = V_old(terminal_obs). The episode
  was cut short; future value should be bootstrapped from the critic.

Getting this wrong biases the entire advantage estimate. We store the
terminal observation and the done/truncated flag per trajectory to handle
this correctly.

## Training loop

PPO reuses rollout data for multiple optimization passes (unlike REINFORCE
which uses each rollout exactly once). This is the key efficiency gain.

```
For each epoch:
  1. Collect rollouts: reset_all(), run 50 envs to completion
     Record per step: (obs, action, reward, mu_old, V_old)
     Record per env: terminal_obs, done_flag (for GAE bootstrap)
  2. Compute GAE advantages using V_old (ONCE — frozen for all K passes)
  3. Compute value targets: R_t = A_t + V_old(s_t) (also frozen)
  4. Normalize advantages (zero-mean, unit-var)
  5. For K optimization passes over the FROZEN rollout data:
     a. Recompute mu_new from current actor params (policy changes each pass)
     b. Recompute V_new from current critic params (critic changes each pass)
     c. Compute importance ratio pi_new/pi_old using mu_new vs mu_old
     d. Compute clipped surrogate gradient (actor) using frozen advantages
     e. Compute value loss gradient (critic) using frozen R_targets
     f. Adam update for both actor and critic
  6. Anneal sigma
```

**Frozen advantages**: GAE advantages and value targets are computed once
from the collected rollout. They do NOT change during the K optimization
passes. Only the policy (mu_new) and value function (V_new) change. This
is standard PPO — recomputing advantages each pass would defeat the purpose
of the clipped objective.

**K optimization passes** is what makes PPO sample-efficient. REINFORCE
uses each rollout once; PPO squeezes K=4 gradient updates from it. This is
safe because the clipped objective prevents the policy from drifting too far
from the behavior that generated the data.

## Hyperparameters

| Param | Value | Rationale |
|---|---|---|
| NUM_ENVS | 50 | Same as CEM and REINFORCE |
| MAX_EPOCHS | 30 | Same budget |
| gamma | 0.99 | Same as REINFORCE |
| gae_lambda | 0.95 | Standard GAE — balances bias/variance |
| clip_eps | 0.2 | Standard PPO clip range |
| K (opt passes) | 4 | Standard PPO mini-batch reuse |
| lr_actor | 0.01 | Lower than REINFORCE — multiple passes amplify effective lr |
| lr_critic | 0.03 | Critic learns faster (simpler function, supervised loss) |
| sigma_init | 0.5 | Same as REINFORCE |
| sigma_min | 0.05 | Same |
| sigma_decay | 0.90 | Same |
| max_grad_norm | 0.5 | Tighter clipping — PPO already limits updates via clip_eps |
| Adam beta1/beta2 | 0.9 / 0.999 | Standard |

## Trajectory storage

```rust
struct Trajectory {
    obs: Vec<[f32; 4]>,
    actions: Vec<[f64; 2]>,
    rewards: Vec<f64>,
    mu_old: Vec<[f64; 2]>,    // mean action at collection time
    v_old: Vec<f64>,           // value estimate at collection time
    terminal_obs: Option<[f32; 4]>,  // last obs (for GAE bootstrap)
    done: bool,               // true = done, false = truncated
}
```

The mu_old and v_old are needed for importance ratio and GAE computation.
terminal_obs + done flag are needed for correct GAE bootstrap at the
trajectory boundary. 50 envs x ~300 steps x ~13 values = ~200 KB per epoch.

## What to stress-test (Phase 2)

1. **Critic gradient correct** — finite-difference check on all 5 params
2. **GAE matches discounted returns at lambda=1** — sanity check
3. **Clipping activates** — verify clip_eps=0.2 actually clips in early epochs
4. **PPO converges in 30 epochs** — the main question
5. **PPO beats REINFORCE** — more reaches OR faster reward improvement
6. **Value loss decreases** — critic is actually learning
7. **Importance ratio stays near 1** — clipping prevents drift
8. **Multiple opt passes help** — K=4 better than K=1

Key question: can the critic (5 params, linear) learn a good enough value
function that GAE advantages are meaningfully better than REINFORCE's
normalized returns? If the reward landscape is roughly quadratic in obs,
a linear V should capture the first-order structure.

## Visual story

- **Epoch 1**: chaos, same as REINFORCE
- **Epoch 5-10**: arms swing toward target, noticeably faster than REINFORCE
- **Epoch 15-20**: all 50 arms reach and park at the green sphere
- **HUD**: shows value loss decreasing, clip fraction, reward curve smoother

The comparison to REINFORCE should be visible: PPO gets to the target region
by epoch 10 (where REINFORCE took 15-20), and actually achieves done
triggers (where REINFORCE plateaued).

## Validation (at epoch 25)

1. **Convergence** — >=10/50 trigger done (what REINFORCE couldn't do)
2. **Reward improvement >= 90%**
3. **Value loss decreased** — critic learned
4. **Clip fraction > 0 in early epochs** — clipping activated

## Risk: linear critic may not be expressive enough

The reward is -(qpos - target)^2, which is quadratic in joint angles. A
linear V(s) = w · s + b can only approximate this as a plane through the
reward landscape. Near the optimum, the quadratic curvature matters and the
linear approximation breaks down.

If stress-testing shows the critic can't learn (value loss doesn't decrease),
the fallback is a quadratic critic: V(s) = s^T M s + w · s + b. This adds
10 params (symmetric 4x4 matrix) for a total of 25, still analytically
tractable.

Only pursue this if the linear critic demonstrably fails in Phase 2.
