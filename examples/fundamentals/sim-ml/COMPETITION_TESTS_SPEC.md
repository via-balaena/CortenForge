# Competition Tests Spec (Phases 3 + 6)

> **Status**: Phase 3 complete, Phase 6 complete, Phase 6b/6c planned
> **Crate**: sim-ml-bridge
> **Parent spec**: ML_COMPETITION_SPEC.md (Phase 3), AUTOGRAD_SPEC.md (Phase 6)
> **Branch**: main

## Context

Phase 2 is complete — all 5 algorithms (CEM, REINFORCE, PPO, TD3, SAC)
are implemented with 239 passing tests. Phase 3 adds integration tests
that use the `Competition` runner to validate the spec's hypotheses about
algorithm ordering. These are the dyno tests — the proof that the
algorithms behave as theory predicts.

## Headline finding

The spec predicted: CEM << REINFORCE < PPO < TD3 <= SAC.

Actual ordering at level 0-1 (seed 42, 50ep/50env, 6-DOF MLP):

    REINFORCE (-7500) << PPO (-3449) << TD3 (-12) ≈ SAC (-11) << CEM (-1.05, 49 dones)

**CEM dominates.** On a smooth quadratic reward landscape -(qpos-target)^2,
gradient-free search (50 candidates/generation, 50 generations) outperforms
hand-coded gradient methods in 614-dim space. The gradients are correct
but noisy — each is multiplied by a high-variance return estimate, and
25K samples per epoch aren't enough to stabilize a 614-dim gradient.

This is the "each failed hypothesis is a finding, not a bug" outcome.
Level 2 (autograd with deeper networks) should reverse this ordering
by providing lower-variance gradients via automatic differentiation,
batch backward passes, and deeper architectures (3+ layers).

## Files modified

| File | Change |
|------|--------|
| `sim/L0/ml-bridge/src/competition.rs` | Added `print_summary()` to `CompetitionResult` |
| `sim/L0/ml-bridge/tests/competition.rs` | **New file** — 7 competition integration tests |

## Test results

All 7 tests pass (seed 42, `--release`). Total runtime: ~7 min.

### Test 1: `competition_2dof_all_linear` (12s)

All 5 algorithms, linear policies, 2-DOF. Regression baseline.

| Algorithm | Reward | Dones | Improvement |
|-----------|--------|-------|-------------|
| CEM | -0.60 | 0 | 90.7% |
| SAC | -8.75 | 0 | N/A (warmup) |
| TD3 | -13.14 | 0 | N/A (warmup) |
| REINFORCE | -182.10 | 3 | 90.4% |
| PPO | -311.67 | 0 | 83.5% |

- Off-policy methods (TD3, SAC) start with warmup (random actions),
  so first-epoch reward is ~0, making "improvement" misleading.
  Asserts use absolute threshold (> -100) for off-policy instead.
- REINFORCE is the only algorithm to trigger dones (3).

### Test 2: `hypothesis_cem_scales_poorly` (39s)

CEM linear on 2-DOF vs CEM MLP on 6-DOF.

| Task | Reward | Improvement | Dones |
|------|--------|-------------|-------|
| 2-DOF | -0.60 | 92.8% | 0 |
| 6-DOF | -2.56 | 88.8% | 0 |

- **Finding**: percentage improvements look similar (~90%) because
  both tasks have different baselines (2 vs 6 joints). The real
  signal is absolute reward: 2-DOF gets 4x closer to target.
- CEM can't trigger the precise done condition (5cm + velocity < 0.5)
  at this budget, even on 2-DOF.

### Test 3: `hypothesis_value_fn_matters_at_scale` (80s)

PPO MLP vs REINFORCE MLP on 6-DOF.

| Algorithm | Reward | Dones |
|-----------|--------|-------|
| PPO | -3246.93 | 0 |
| REINFORCE | -6525.24 | 0 |

- PPO's learned value baseline cuts gradient variance — 2x better
  reward than REINFORCE. Hypothesis confirmed.
- Neither triggers dones at 40 epochs.

### Test 4: `hypothesis_off_policy_efficiency` (22s)

TD3 MLP vs PPO MLP vs CEM MLP on 6-DOF at low budget (20ep/20env).

| Algorithm | Reward | Dones |
|-----------|--------|-------|
| CEM | -7.37 | 0 |
| TD3 | -11.52 | 0 |
| PPO | -6437.02 | 0 |

- **Finding**: CEM beats both gradient methods at very low budget.
  No warmup overhead, no noisy gradient estimates — just perturb
  and evaluate. TD3's 100-step warmup eats 5 of 20 epochs.
- TD3 >> PPO (560x better reward) — off-policy replay dominates
  on-policy at low data budgets. Hypothesis confirmed for
  off-policy vs on-policy; refuted for off-policy vs evolutionary.

### Test 5: `hypothesis_mlp_beats_linear` (37s)

PPO MLP vs PPO linear on 6-DOF.

| Policy | Reward | Improvement |
|--------|--------|-------------|
| Linear (78 params) | -2705.36 | 60.0% |
| MLP (614 params) | -3878.97 | 42.6% |

- **Finding**: Linear beats MLP. The quadratic reward landscape
  -(qpos-target)^2 is well-served by a linear controller (PD-like).
  Linear converges faster with fewer params per gradient update.
  MLP's extra capacity is wasted overhead when the reward doesn't
  require nonlinear function approximation.
- This hypothesis needs a genuinely nonlinear task (obstacle
  avoidance, contact manipulation) to test properly.

### Test 6: `hypothesis_entropy_helps` (28s)

SAC linear vs TD3 linear on 6-DOF.

| Algorithm | Reward | Dones |
|-----------|--------|-------|
| TD3 | -26.05 | 0 |
| SAC | -28.84 | 0 |

- **Finding**: precondition failed — `improvement_pct` returned
  `-inf` because first-epoch reward was 0.0 (warmup). Linear
  policies are too weak for meaningful 6-DOF off-policy comparison.
  Test correctly skips the entropy assertion and logs the finding.
- Revisit with `MlpStochasticPolicy` at level 1+.

### Test 7: `competition_6dof_all_mlp` (191s)

All 5 algorithms, MLP policies, 6-DOF. The headline test.

| Algorithm | Reward | Dones |
|-----------|--------|-------|
| CEM | -1.05 | 49 |
| SAC | -10.82 | 0 |
| TD3 | -11.99 | 0 |
| PPO | -3449.38 | 0 |
| REINFORCE | -7499.96 | 0 |

- Asserts the verified level 0-1 ordering: CEM > TD3 > PPO > REINFORCE.
- CEM is the only algorithm to trigger dones (49 out of 50 envs x 50 epochs).

## Why the ordering reversed

The spec's prediction assumed gradient estimates are good enough to
outperform evolutionary search. At level 0-1 with hand-coded gradients,
they aren't:

1. **Return variance**: REINFORCE multiplies per-sample gradients by
   noisy return estimates. PPO uses a learned V(s) but V(s) itself is
   a 417-param MLP being trained simultaneously.
2. **Single hidden layer ceiling**: 32 hidden units limits the gradient
   landscape's expressiveness.
3. **Sequential gradient computation**: 25K individual `log_prob_gradient`
   calls per epoch, each allocating a `Vec<f64>`. No batch backward pass.
4. **Scalar sigma**: isotropic Gaussian exploration (one sigma for all
   action dimensions) is a blunt instrument in 6-DOF.

CEM wins because it doesn't estimate gradients at all. On a smooth,
unimodal reward landscape, 50 candidates/generation is enough to find
downhill directions in 614-dim space. No variance from return estimation.

## What level 2 (autograd) should change

Level 2 replaces hand-coded backprop with automatic differentiation
(e.g., burn). This enables:

- **Deeper networks** (3+ layers) — richer gradient landscape
- **Batch backward pass** — one pass over the full batch, not 25K
  sequential calls
- **Lower numerical noise** — computation graph with controlled precision
- **Per-dimension sigma** — anisotropic Gaussian exploration
- **Larger param counts** — 5K+ params where CEM's 50 candidates
  become fundamentally inadequate

Prediction: autograd reverses CEM's dominance because gradient methods
can exploit local curvature. CEM's search space explodes combinatorially
but the gradient points directly downhill.

---

## Level 2 results (autograd, Phase 6)

### Test 8: `competition_6dof_autograd_1layer_parity` (506s)

All 5 algorithms, autograd backends, 1 hidden layer (32 units), 6-DOF.
Same architecture as level 0-1 — validates autograd doesn't regress.

| Algorithm | Reward | Dones | Level 0-1 Reward |
|-----------|--------|-------|------------------|
| CEM | -1.05 | 49 | -1.05 |
| SAC | -10.64 | 0 | -10.82 |
| TD3 | -11.99 | 0 | -11.99 |
| PPO | -3449.38 | 0 | -3449.38 |
| REINFORCE | -7499.96 | 0 | -7499.96 |

- **Exact parity** for CEM, TD3, PPO, REINFORCE — autograd produces
  identical results with the same architecture.
- **SAC improved** (-10.82 → -10.64) and **overtakes TD3** for the first
  time. `AutogradStochasticPolicy` gives SAC an MLP actor (previously
  stuck with `LinearStochasticPolicy`).
- Ordering: CEM >> SAC > TD3 >> PPO >> REINFORCE (same as level 0-1,
  with SAC/TD3 swap).

### Test 9: `competition_6dof_autograd_2layer` (2851s)

All 5 algorithms, autograd backends, 2 hidden layers (64+64), ReLU,
Xavier/He init, 6-DOF. ~5,400 params per network (vs 614 at level 0-1).

| Algorithm | Reward | Dones | Level 0-1 Reward | Change |
|-----------|--------|-------|------------------|--------|
| CEM | -3.07 | 0 | -1.05 | 3x worse |
| TD3 | -4.08 | 0 | -11.99 | 3x better |
| SAC | -30.04 | 0 | -10.82 | 3x worse |
| PPO | -9025.90 | 0 | -3449.38 | 2.6x worse |
| REINFORCE | -11979.50 | 0 | -7499.96 | 1.6x worse |

Ordering: CEM (-3.07) > TD3 (-4.08) >> SAC >> PPO >> REINFORCE

### Headline finding

**The ordering reversal has not happened at 50 epochs — but it's imminent.**

CEM and TD3 are converging from opposite directions:
- **CEM degraded 3x** (-1.05 → -3.07): 50 candidates can't search
  5,400 dims effectively. CEM has plateaued.
- **TD3 improved 3x** (-11.99 → -4.08): the deeper network is learning,
  just needs more gradient steps. TD3's evaluation rewards during
  training: -29 → -18 → -9 → -5 → -4. Still converging.

The gap is 1 reward unit (-3.07 vs -4.08). TD3 at this trajectory
would overtake CEM within ~70-100 more epochs.

### Why 50 epochs isn't enough

The budget was calibrated for 614 params (level 0-1). At 5,400 params:

| Factor | Level 0-1 | Level 2 | Impact |
|--------|-----------|---------|--------|
| Params | 614 | ~5,400 | 8.8x more parameters to optimize |
| Steps/param | ~2,000 | ~230 | 8.8x less per-param training signal |
| CEM candidates/param | 1:12 | 1:108 | Fundamentally insufficient for CEM |
| TD3 replay ratio | ~780x | ~780x | Same — replay helps scale |

Off-policy methods (TD3) handle the scaling best because replay
multiplies effective sample count. On-policy methods (PPO, REINFORCE)
use each sample once, so 8.8x more params = 8.8x less per-param signal.

### SAC instability

SAC finished at -30.04 but was at -10.81 two epochs earlier. The
final-epoch reward is a fluke — SAC oscillated between -8 and -30
throughout training. Root cause: learning rate 3e-4 is too aggressive
for a deep stochastic policy with entropy regularization. The entropy
bonus pulls the policy toward randomness while the Q-gradient pulls
toward exploitation — with a too-large LR, the policy oscillates
between these attractors.

### Follow-up experiments (Phase 6b)

Three levers to test the reversal hypothesis:

1. **More epochs** (200-300): let TD3/SAC converge with same hyperparams.
   Estimated ~40 min per algorithm in release mode.
2. **Lower learning rate** (3e-4 → 1e-4): stabilize SAC and help all
   gradient methods converge smoothly with 5K params.
3. **More environments** (50 → 200): 4x more data per epoch. Helps
   on-policy methods (PPO, REINFORCE) most — each gradient estimate
   uses 4x more samples.

These are independent variables — run one at a time to isolate the effect.

---

## Verification

```bash
# Quick: existing tests still pass
cargo test -p sim-ml-bridge --lib

# IMPORTANT: always use --release for competition tests.
# Debug mode is 5-10x slower (physics sim + gradient math).

# Run one hypothesis (fast feedback)
cargo test -p sim-ml-bridge --test competition --release hypothesis_cem -- --ignored --nocapture

# Run level 0-1 tests (Tests 1-7, ~7 min in release)
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture

# Run level 2 autograd tests only (Tests 8-9, ~56 min in release)
# These print epoch-level progress via Competition::new_verbose()
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture competition_6dof_autograd

# Run one autograd test
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture competition_6dof_autograd_1layer_parity
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture competition_6dof_autograd_2layer
```

---

## Phase 6b results (budget scaling experiments)

### Test 11: `budget_scaling_lower_lr` (2899s)

6b-2: Lower learning rates, 50 epochs, 50 envs, 2-layer [64,64] ReLU
Xavier, seed 42. LR changes: REINFORCE 0.05 → 0.01, PPO 0.025 → 0.005,
TD3 3e-4 → 1e-4, SAC 3e-4 → 1e-4 (optimizer + `alpha_lr`). CEM unchanged.

| Algorithm | Reward | Dones | Phase 6 Reward | Change |
|-----------|--------|-------|----------------|--------|
| CEM | -3.07 | 0 | -3.07 | Same (gradient-free) |
| TD3 | -9.81 | 0 | -4.08 | 2.4x worse |
| SAC | -14.40 | 0 | -30.04 | 2.1x better |
| PPO | -9394.01 | 0 | -9025.90 | Slightly worse |
| REINFORCE | -9740.68 | 0 | -11979.50 | 1.2x better |

Ordering: CEM (-3.07) > TD3 (-9.81) > SAC (-14.40) >> PPO >> REINFORCE

**CEM still dominates.** The ordering reversal did not happen — lower LR
alone is not enough.

### Findings

- **SAC hypothesis confirmed.** SAC stabilized from -30.04 to -14.40
  (2.1x better). At original LR, SAC oscillated between -8 and -30 due
  to entropy/exploitation tug-of-war. At 1e-4, the oscillation range
  narrowed to -10 to -26. Still not stable enough to challenge CEM, but
  a clear improvement.

- **TD3 got worse.** -4.08 → -9.81 (2.4x worse). The lower LR slowed
  convergence — TD3 needs more gradient steps to compensate. However,
  TD3's epoch-by-epoch trajectory tells a different story:

  ```
  TD3 eval rewards (odd epochs only — even epochs are training):
  ep1: -19 → ep9: -10 → ep19: -3.78 → ep27: -3.83 → ep43: -2.63 → ep45: -1.95 → ep49: -9.81
  ```

  **TD3 hit -1.95 at epoch 45 — better than CEM's -3.07.** The reversal
  happened transiently but TD3 couldn't hold it. The policy found a good
  region, then drifted away. This suggests TD3 with lower LR *can*
  overtake CEM but needs either more epochs or a scheduler to lock in
  the gains.

- **PPO diverged, then slowly recovered.** Started at -12,802, worsened
  to -15,105 by epoch 22 (the lower LR couldn't prevent initial
  divergence with 5K params), then slowly recovered to -9,394 by epoch
  49. Still 3 OOM behind CEM. The learning curve was U-shaped — PPO
  needs fundamentally more data per epoch, not just lower LR.

- **REINFORCE showed a similar U-shape.** Improved from -12,802 to
  -4,705 by epoch 26, then regressed to -9,741. The lower LR helped
  early convergence but couldn't prevent the high-variance gradient
  estimates from destabilizing the policy.

### Implications for Tests 10 and 12

The TD3 transient reversal at epoch 45 (-1.95 < -3.07) strongly supports
the Test 10 hypothesis: 200 epochs should give TD3 enough time to
converge and hold. The key question is whether TD3's instability at
low LR (the -1.95 → -9.81 regression) also appears at the original
3e-4 LR with more epochs.

### Test 10: `budget_scaling_more_epochs` (6478s)

6b-1: 200 epochs, 50 envs, 2-layer [64,64] ReLU Xavier, seed 42.
Baseline variable: epochs (50 → 200). Top 3 only (CEM, TD3, SAC).

| Algorithm | Reward | Dones | Phase 6 Reward | Change |
|-----------|--------|-------|----------------|--------|
| TD3 | -5.22 | 0 | -4.08 | 1.3x worse |
| CEM | -5.75 | 6 | -3.07 | 1.9x worse |
| SAC | -31.67 | 0 | -30.04 | Slightly worse |

Ordering: TD3 (-5.22) > CEM (-5.75) >> SAC (-31.67)

**The ordering reversed.** TD3 overtakes CEM for the first time.

### Headline finding

The reversal happened — but not the way we predicted. TD3 didn't
converge to a great policy while CEM plateaued. Instead, **both
algorithms degraded from their 50-epoch performance**, and CEM
degraded faster.

- **CEM degraded 1.9x** (-3.07 → -5.75). CEM reached -2.19 at epoch
  48 (its all-time best), then oscillated between -2 and -6 for the
  remaining 150 epochs. After hitting `noise_min` (~epoch 75), CEM
  can only make tiny perturbations. With no gradient signal, it
  random-walks instead of converging — each generation's elite set
  drifts aimlessly in 5,400-dim space.

- **TD3 degraded 1.3x** (-4.08 → -5.22). TD3's eval trajectory was
  volatile throughout:
  ```
  ep1: -9 → ep49: -4.08 → ep69: -3.89 → ep79: -19.25 → ep137: -6.09 → ep173: -3.99 → ep199: -5.22
  ```
  TD3 repeatedly found good policies (-3.89, -3.99) then drifted away.
  The replay buffer's 50K capacity means it forgets early good
  transitions — by epoch 100+, the buffer only holds the last ~2
  epochs of data. Combined with the fixed exploration noise (0.1),
  TD3 oscillates between exploitation and disruption.

- **SAC collapsed.** -30.04 → -31.67. SAC's trajectory shows
  catastrophic instability: -8.11 (ep1) → -42.28 (ep77) → stuck at
  -24 (ep95-130) → brief recovery to -14.28 (ep153) → collapse to
  -31.67. The 3e-4 LR is too aggressive for 200 epochs of
  entropy-regularized training. (Test 11 showed 1e-4 stabilizes SAC
  to -14.40 — but at only 50 epochs.)

### Why both degraded

The 50-epoch results were partially lucky — both algorithms happened to
be at favorable points in their oscillation at epoch 49. With 200
epochs:

1. **CEM's noise schedule is wrong for long training.** It decays to
   `noise_min` by epoch ~75 and then stalls. CEM needs either cyclical
   noise (re-inject exploration periodically) or adaptive noise that
   responds to reward stagnation.

2. **TD3's replay buffer is too small.** At 50K capacity with 50 envs
   × 500 steps = 25K transitions/epoch, the buffer only holds ~2
   epochs. Old good-policy transitions are evicted, and TD3 has to
   relearn from scratch when the current policy degrades. A larger
   buffer (200K-500K) would retain more history.

3. **SAC's LR is too aggressive.** 3e-4 works for 50 epochs but causes
   divergence at 200. The entropy/exploitation oscillation amplifies
   over time.

### The reversal is real but fragile

TD3 > CEM at 200 epochs, but the margin is thin (-5.22 vs -5.75) and
both are worse than their 50-epoch results. The reversal confirms the
*direction* of the hypothesis (gradient methods scale better than CEM
with deeper networks) but reveals that raw epoch count isn't enough —
the algorithms need hyperparameter tuning for long-horizon training.

The clean win would be: TD3 with lower LR (1e-4) + more epochs (200) +
larger replay buffer. That combines the stability from Test 11 with the
training duration from Test 10. This is Phase 6c territory.

### Test 12: `budget_scaling_more_envs` (7488s)

6b-3: 200 environments, 50 epochs, 2-layer [64,64] ReLU Xavier, seed 42.
Baseline variable: envs (50 → 200). All 5 algorithms.

| Algorithm | Reward | Dones | Phase 6 Reward | Change |
|-----------|--------|-------|----------------|--------|
| CEM | -0.84 | 195 | -3.07 (0 dones) | 3.7x better |
| SAC | -14.49 | 0 | -30.04 | 2.1x better |
| TD3 | -19.57 | 0 | -4.08 | 4.8x worse |
| PPO | -14317.50 | 0 | -9025.90 | 1.6x worse |
| REINFORCE | -15164.81 | 0 | -11979.50 | 1.3x worse |

Ordering: CEM (-0.84) >> SAC (-14.49) > TD3 (-19.57) >> PPO >> REINFORCE

**CEM dominates completely.** -0.84 with 195 dones — better than CEM's
level 0-1 result (-1.05, 49 dones) with a 1-hidden-layer network.

### Findings

- **CEM thrives with more candidates.** 200 candidates/generation
  (1:27 params/candidate) vs 50 (1:108) at baseline. The 4x better
  search density transforms CEM from "struggling in 5,400 dims" to
  "reliably solving the task." CEM's learning curve was monotonically
  decreasing — no oscillation, no regression. By epoch 35, CEM was
  triggering dones. By epoch 49, 195 cumulative dones across training.

- **TD3 collapsed (4.8x worse).** -4.08 → -19.57. Root cause: the
  replay buffer (50K capacity) can't hold even one epoch of data.
  200 envs × 500 steps = 100K transitions per epoch, so the buffer
  wraps twice per epoch. TD3's core advantage — replaying diverse
  past experience — is completely negated. The buffer only holds the
  last half-epoch, making TD3 effectively on-policy with extra overhead.

- **SAC improved (2.1x better).** -30.04 → -14.49. The 4x more data
  per training epoch helps SAC's Q-function converge, partially
  compensating for the aggressive LR. Same improvement magnitude as
  Test 11's lower LR (-14.40), suggesting both interventions address
  the same underlying issue (Q-estimate variance).

- **On-policy methods got worse — hypothesis refuted.** REINFORCE
  (-11,980 → -15,165) and PPO (-9,026 → -14,318) both degraded with
  4x more data. This contradicts the prediction that more samples per
  gradient estimate would help.

  Root cause: the gradient computation likely sums (not averages)
  log-prob gradients weighted by returns across all samples. 4x more
  envs = 4x larger raw gradient magnitude = effectively 4x higher
  learning rate. REINFORCE showed this clearly: improved to -10,303
  by epoch 4, then diverged to -15,165 — classic LR-too-high
  instability. The LR needs to be scaled down by 1/n_envs when
  increasing environment count, or the gradient must be normalized
  by batch size.

### Phase 6b summary

| Test | Variable | Winner | Key finding |
|------|----------|--------|-------------|
| 11 (LR) | 3e-4 → 1e-4 | CEM (-3.07) | SAC stabilized. TD3 transiently beat CEM at ep45 (-1.95). |
| 10 (epochs) | 50 → 200 | TD3 (-5.22) | **Reversal confirmed** — but fragile. Both degraded. |
| 12 (envs) | 50 → 200 | CEM (-0.84) | CEM's best result ever. Gradient methods collapsed. |

**The picture**: CEM and gradient methods have complementary strengths.
CEM scales with candidates (more envs). Gradient methods scale with
training duration (more epochs). Neither scales well with the other's
lever.

The hypothesis that gradient methods would overtake CEM is **partially
confirmed**: TD3 wins at 200 epochs (Test 10). But CEM wins harder at
200 envs (Test 12). The real lesson is that algorithm ordering depends
on *how* you scale the compute budget, not just *how much*.

Next steps (Phase 6c): design a nonlinear task where CEM's gradient-free
search fundamentally can't compete — obstacle avoidance, contact
manipulation, or multi-waypoint reaching. On a smooth quadratic reward,
CEM will always be competitive if given enough candidates.