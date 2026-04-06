# Auto-Reset — Reaching Arm + CEM

Fifty two-link arms learn to reach a green target using the Cross-Entropy
Method (CEM). Each generation, the VecEnv runs 50 policy perturbations in
parallel. Arms that reach the target trigger `done` and auto-reset; arms
that time out trigger `truncated` and auto-reset. After all 50 finish, CEM
selects the top 15 performers, updates the policy distribution, and the
next generation begins. This is the first example where the sim is actually
*learning* — you watch a population go from chaos to coordination.

See also: [Parallel Stepping](../parallel-step/) — the simpler VecEnv demo
(constant actions, no learning, no resets).

## What you see

- **50 arms in a row**, each a two-link planar arm (steel-blue upper arm,
  warm-orange forearm). All share one physics Model and step in parallel
  via `VecEnv::step()`
- **Green spheres** mark the target position (0.4, 0, 0.3) — up and to the
  right of each shoulder. The arm must fight gravity to reach it
- **Generation 1: chaos** — random policy perturbations produce wild
  flailing. Arms swing in every direction, most time out at 3 seconds
- **Generations 10–15: roughly right** — the CEM mean has shifted. Many
  arms swing toward the target region but overshoot or oscillate
- **Generations 25–30: convergence** — 18–23 of 50 arms reach within 5 cm
  of the target with velocity < 0.5 rad/s, triggering `done` and
  auto-resetting. The rest time out (policy perturbations that happened
  to be poor this generation)
- **UPDATING pause** — a 1.5-second freeze between generations where the
  HUD shows "Updating." This is the CEM distribution update — honest
  wall-clock time representing the computation that happens between
  rollout batches in real RL training
- **HUD** (bottom-left) — generation counter, phase, reach count, best
  reward, sigma mean, and a per-env scoreboard showing the top 8 envs
  sorted by cumulative reward. Stars mark elites

## How it works

### Physics

A two-link planar arm in the XZ plane. Shoulder hinge at the origin, both
links extend along +X at qpos=(0,0). Under gravity the arm swings down and
hangs. Each episode starts horizontal.

| Parameter | Value |
|-----------|-------|
| Upper arm | L=0.5 m, mass=0.5 kg, damping=2.0 |
| Forearm | L=0.4 m, mass=0.3 kg, damping=1.0 |
| Shoulder motor | gear=10, ctrl range [-1, 1] |
| Elbow motor | gear=5, ctrl range [-1, 1] |
| Target | (0.4, 0, 0.3) — IK solution: qpos=(-0.242, 1.982) |
| Integrator | RK4, dt=0.002 s, no contacts |

Gear ratios are deliberately low (10/5). With gear=10, gravity compensation
at the shoulder requires ctrl ≈ 0.25 — well within CEM's search range.
Higher gear ratios push the required ctrl toward ~0.05, which CEM can't
reliably distinguish from zero.

### Policy

Linear with observation normalization:
**action = tanh(W * obs_scaled + b)**

| Component | Detail |
|-----------|--------|
| obs | qpos[0], qpos[1], qvel[0], qvel[1] — 4 dims |
| obs_scaled | obs * [1/pi, 1/pi, 0.1, 0.1] — normalizes to ~[-1, 1] |
| W | [2x4] = 8 parameters |
| b | [2] = 2 parameters |
| Total | 10 parameters |

Observation normalization prevents tanh saturation: without it, qvel values
(~10 rad/s during swings) produce pre-tanh values of ~10 that fully
saturate, making all perturbations output the same +/-1 and blinding CEM.

### CEM

| Parameter | Value | Why |
|-----------|-------|-----|
| Envs (N) | 50 | ~5x param count — the minimum for reliable convergence on this problem |
| Elites (K) | 15 | Top 30%, balances exploitation and exploration |
| Generations | 30 | Sufficient for convergence across all tested seeds |
| sigma_init | 1.0 | Wide initial exploration |
| sigma_min | 0.1 | Prevents premature sigma collapse |

**Reward**: joint-space squared error — `-(qpos[0] - target_q[0])^2 - (qpos[1] - target_q[1])^2`. Joint-space provides a smooth, convex objective
that CEM can optimize directly. Cartesian end-effector distance creates a
reward plateau at ~27 cm that CEM can't escape.

**Done**: Cartesian end-effector within 5 cm of target AND velocity < 0.5
rad/s. This preserves the visual story of "reaching the green sphere."

**Truncated**: episode time > 3.0 s.

### VecEnv features exercised

| Feature | How |
|---------|-----|
| `done` auto-reset | Arm reaches target → episode ends → arm resets to horizontal |
| `truncated` auto-reset | 3s timeout → episode ends → arm resets |
| `reward_fn` | Joint-space error drives CEM elite selection |
| `terminal_observations` | Populated on every done/truncated |
| `sub_steps(5)` | 100 Hz control rate (5 physics steps per action) |
| Async episode lengths | Good arms finish early, bad arms timeout |
| `reset_all()` | Full population reset between generations |

## Validation (at generation 25)

| Check | Expected |
|-------|----------|
| Convergence | >= 10/50 envs trigger `done` |
| Reward improvement | >= 50% from generation 1 |
| Policy shifted | mu norm > 0.5 |
| Sigma decreased | mean(sigma) < 0.5 |

## Run

```
cargo run -p example-ml-vec-env-auto-reset --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
