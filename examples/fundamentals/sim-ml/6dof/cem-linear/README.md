# 6-DOF CEM + Linear Policy

Fifty three-segment arms learn to reach a target using Cross-Entropy Method
(CEM) with a linear policy. Same CEM algorithm as the 2-DOF auto-reset
example, but the arm now has 3 segments, 6 hinge joints, and the policy
grows from 10 parameters to 78.

The visual story: CEM was king on 2-DOF because 10 parameters is a small
search space. On 6-DOF, 78 parameters is still tractable for sampling-based
search — you'll see some arms converge, but fewer than the 2-DOF example.
Phase 5 continues with PPO+MLP (#20) and SAC+autograd (#21) to show why
gradient methods and deeper networks matter as the search space grows.

See also: [Auto-Reset (2-DOF CEM)](../../vec-env/auto-reset/) — same
algorithm, simpler arm, smaller search space.

## What you see

- **50 arms in a row**, each a three-segment planar arm:
  - **seg1** (steel-blue) — longest, base joints (j1 pitch, j2 yaw)
  - **seg2** (cyan) — middle segment (j3 pitch, j4 yaw)
  - **seg3** (warm-orange) — shortest, fingertip end (j5 pitch, j6 yaw)
- **Green spheres** mark the target (computed by FK from
  `target_joints = [0.5, 0.2, -0.8, 0.1, 0.5, -0.1]`)
- **Generation 1** — random perturbations, arms flail in all directions
- **Generations 10–20** — some arms find partial solutions (seg1 aims
  correctly, seg2/seg3 still searching)
- **Generations 30–40** — several arms coordinate all 3 segments and reach
  the target within 5cm + velocity <1.0 rad/s
- **UPDATING pause** — 1.5s freeze between generations for the CEM update
- **HUD** (bottom-left) — generation, phase, reach count, best reward,
  sigma, param count, per-env scoreboard

## How it works

### Physics

Three-segment planar arm with 6 hinge joints (alternating pitch/yaw),
decreasing gear ratios along the chain (10→8→6→5→4→3).

| Segment | Length | Joints | Damping | Motor gears |
|---------|--------|--------|---------|-------------|
| seg1 | 0.30 m | j1 pitch, j2 yaw | 2.0, 1.5 | 10, 8 |
| seg2 | 0.25 m | j3 pitch, j4 yaw | 1.5, 1.0 | 6, 5 |
| seg3 | 0.20 m | j5 pitch, j6 yaw | 1.0, 0.5 | 4, 3 |

Integrator: RK4, dt=0.002s, sub_steps=5 (100 Hz control rate), no contacts.

### Policy

Linear: **action = tanh(W · obs_scaled + b)**, **78 params** (W[6×12] + b[6]).

| Component | Detail |
|-----------|--------|
| obs | 12-dim (6 qpos + 6 qvel) |
| obs_scale | [1/π×6, 0.1×6] |
| W | 6×12 = 72 weights |
| b | 6 biases |
| Total | 78 parameters |

### CEM config

| Parameter | Value | Why |
|-----------|-------|-----|
| Envs (N) | 50 | ~0.6× param count — tight but sufficient |
| Elites (K) | 10 | Top 20% |
| Generations | 40 | More than 2-DOF (30) because search space is larger |
| sigma_init | 1.0 | Wide initial exploration |
| sigma_min | 0.1 | Floor to prevent collapse |

**Reward**: `-Σ(qpos[j] - target_joints[j])²` over all 6 joints.
**Done**: fingertip within 5cm of target AND all joint velocities < 1.0 rad/s.
**Truncated**: episode time > 5.0s.

### 2-DOF vs 6-DOF CEM

| | 2-DOF | 6-DOF |
|---|-------|-------|
| Params | 10 | 78 |
| Envs | 50 | 50 |
| Params/env ratio | 5× | 0.6× |
| Generations to converge | ~25 | ~30–40 |
| Typical reach count | ~20/50 | ~5–10/50 |

CEM's sampling efficiency degrades as param count grows. At 78 params, 50
envs per generation is on the edge of what CEM can handle. This is
exactly why the next examples introduce gradient methods (#20) and deeper
networks (#21) — gradient information is more efficient than sampling when
the search space is large.

## Validation (at generation 30)

| Check | Expected |
|-------|----------|
| Some arms reach target | ≥5/50 |
| Reward improvement | ≥30% from gen 1 |
| Policy shifted | ‖μ‖ > 0.5 |
| σ decreased | mean(σ) < 0.5 |

## Run

```
cargo run -p example-ml-6dof-cem-linear --release
```

Orbit: left-drag | Pan: right-drag | Zoom: scroll
