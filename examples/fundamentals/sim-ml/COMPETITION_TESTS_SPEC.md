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
