# Phase 6c — Obstacle Avoidance Task

> **Status**: In progress — 6c-0 and 6c-1 complete, 6c-2 and 6c-3 remaining
> **Crate**: sim-ml-bridge
> **Parent spec**: AUTOGRAD_SPEC.md (Phase 6c), COMPETITION_TESTS_SPEC.md
> **Branch**: feature/phase-6c

---

## 1. Problem statement

Every competition test so far uses `reaching_6dof` — a smooth, convex,
unimodal reward landscape (`-Σ(qpos - target)²`). This landscape is
nearly optimal for CEM: random search on a quadratic bowl works almost
as well as gradient descent, and a linear controller is sufficient
because the mapping from observation to optimal action is approximately
linear.

Phase 6c needs a task that **structurally advantages gradient methods
and deep networks** over CEM and linear controllers.

---

## 2. Task design: obstacle avoidance reaching

Same 6-DOF arm. Same target. A static obstacle (sphere) sits between
the arm's rest configuration and the target. The agent must curve
around it.

### Why this breaks CEM and linear controllers

1. **Nonlinear obs→action mapping.** The optimal action depends on
   *where* the fingertip is relative to the obstacle — "go left" when
   approaching from the right, "go up" when approaching from below.
   A linear controller can only learn one fixed direction of avoidance.

2. **Multi-modal reward landscape.** Two paths around the obstacle
   (above and below, or left and right). CEM's Gaussian perturbations
   smear across both paths. Gradient methods can commit to one path
   and follow the curvature.

3. **Task-space reward.** The reward is based on fingertip position
   (end-effector), not joint angles. The mapping from joint angles to
   fingertip position is nonlinear (forward kinematics). An MLP can
   learn this; a linear policy can only approximate it locally.

4. **Penalty discontinuity.** The obstacle penalty creates a
   non-convex reward surface — there's a penalty ridge through the
   middle of the workspace that the policy must learn to avoid entirely.

### MJCF

Same 6-DOF arm as `reaching_6dof`, with three changes:

- **`fusestatic="false"`** — preserves the static obstacle body as a
  distinct body with its own `xpos` entry (default `fusestatic="true"`
  would merge it into worldbody and delete its body index)
- **Obstacle body** — a static sphere positioned between the arm's
  rest position and the target fingertip position
- **Target site** — a `<site>` on the worldbody marking the target
  fingertip position, observable via `SiteXpos`

Contacts remain disabled (`<flag contact="disable"/>`). The obstacle
is a distance-penalty ghost — the policy sees it and is penalized for
proximity, but no contact forces are computed. This keeps the physics
identical to `reaching_6dof` and isolates the reward nonlinearity as
the only new variable.

```xml
<mujoco model="obstacle-reaching-6dof">
  <compiler angle="radian" inertiafromgeom="true" fusestatic="false"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag contact="disable"/>
  </option>
  <default>
    <geom contype="0" conaffinity="0"/>
  </default>
  <worldbody>
    <site name="target" pos="0.681474 0.154033 0.101028" size="0.015"/>
    <!-- Same 3-segment arm as reaching_6dof -->
    <body name="seg1" pos="0 0 0">
      <joint name="j1" type="hinge" axis="0 -1 0" damping="2.0"
             limited="true" range="-3.14 3.14"/>
      <joint name="j2" type="hinge" axis="0 0 1" damping="1.5"
             limited="true" range="-1.57 1.57"/>
      <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03" mass="0.5"/>
      <body name="seg2" pos="0.3 0 0">
        <joint name="j3" type="hinge" axis="0 -1 0" damping="1.5"
               limited="true" range="-2.6 2.6"/>
        <joint name="j4" type="hinge" axis="0 0 1" damping="1.0"
               limited="true" range="-1.57 1.57"/>
        <geom type="capsule" fromto="0 0 0 0.25 0 0" size="0.025" mass="0.3"/>
        <body name="seg3" pos="0.25 0 0">
          <joint name="j5" type="hinge" axis="0 -1 0" damping="1.0"
                 limited="true" range="-2.6 2.6"/>
          <joint name="j6" type="hinge" axis="0 0 1" damping="0.5"
                 limited="true" range="-1.57 1.57"/>
          <geom type="capsule" fromto="0 0 0 0.2 0 0" size="0.02" mass="0.2"/>
          <site name="fingertip" pos="0.2 0 0" size="0.015"/>
        </body>
      </body>
    </body>
    <!-- Static obstacle (ghost — distance penalty only, no contacts) -->
    <body name="obstacle" pos="0.730 0.046 0.030">
      <geom name="obstacle" type="sphere" size="0.06"
            contype="0" conaffinity="0" mass="0"/>
    </body>
  </worldbody>
  <actuator>
    <motor joint="j1" gear="10" ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j2" gear="8"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j3" gear="6"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j4" gear="5"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j5" gear="4"  ctrllimited="true" ctrlrange="-1 1"/>
    <motor joint="j6" gear="3"  ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>
</mujoco>
```

**Body/site indices** (depth-first document order, verified against
`sim/L0/mjcf/src/builder/body.rs:103`):

| Index | Name | Type |
|-------|------|------|
| body 0 | world | worldbody |
| body 1 | seg1 | arm segment |
| body 2 | seg2 | arm segment |
| body 3 | seg3 | arm segment (fingertip parent) |
| body 4 | obstacle | static sphere |
| site 0 | target | worldbody site (target position) |
| site 1 | fingertip | seg3 site (end-effector) |

**Note on site ordering**: Sites on the worldbody are processed before
child bodies (`builder/mod.rs:280`), so the target site (on worldbody)
gets index 0, and the fingertip site (on seg3) gets index 1.

**Obstacle placement** (RESOLVED — FK analysis complete):

FK results:
- Rest-state fingertip (qpos=0): (0.750, 0.000, 0.000)
- Target fingertip (qpos=[0.5,0.2,-0.8,0.1,0.5,-0.1]): (0.681, 0.154, 0.101)
- Rest→target distance: 0.197 m

Obstacle placed at 30% of rest→target path: **(0.730, 0.046, 0.030)**.

Midpoint (50%) was too close to target — fingertip-to-obstacle distance
(0.098m) was less than `r_safe` (0.12m), causing penalty to fire at the
goal. Moving to 30% gives:
- Distance from target fingertip: 0.138m > `r_safe` (penalty = 0 at goal)
- Distance from rest fingertip: 0.058m < `r_safe` (penalty fires at rest)

The obstacle radius (0.06 m) with `r_safe=0.12` creates a penalty zone
that fully blocks the straight-line path while leaving detour paths
feasible within joint limits.

### Observation space

21 dimensions. Explicit fingertip, obstacle, and target positions give
the policy full spatial information. This is a fairer test: even with
perfect information, a linear controller can't represent "go around."

| Element | Source | Dim | Scale | Rationale |
|---------|--------|-----|-------|-----------|
| qpos | `data.qpos[0..6]` | 6 | 1/π | Joint positions |
| qvel | `data.qvel[0..6]` | 6 | 0.1 | Joint velocities |
| fingertip pos | `data.site_xpos[1]` via `SiteXpos(1..2)` | 3 | 1.0 | True end-effector position |
| obstacle pos | `data.xpos[4]` via `Xpos(4..5)` | 3 | 1.0 | Obstacle center |
| target pos | `data.site_xpos[0]` via `SiteXpos(0..1)` | 3 | 1.0 | Target position |

**Fingertip precision**: Uses the new `SiteXpos` extractor (Phase 6c-0)
to observe the true fingertip site position — not the seg3 body center,
which is 0.2m away (the full segment length). The reward/done closures
also use `data.site_xpos[1]` for consistency.

**Obstacle observation**: Uses `Xpos(4..5)` since the obstacle is a
real body in the MJCF. Its `xpos` is computed during FK (verified:
static bodies with `body_jnt_num=0` get correct positions via
`position.rs:30-157`).

**Target observation**: Uses `SiteXpos(0..1)` since the target is a
site on the worldbody. Static, but included in the observation so
the policy has explicit goal information.

### Action space

Same as `reaching_6dof`: 6 motor torques, ctrl-limited to [-1, 1].

### Reward function

```
reward = -dist(fingertip, target)
         - λ * max(0, r_safe - dist(fingertip, obstacle))
```

| Term | Purpose | Value |
|------|---------|-------|
| `-dist(fingertip, target)` | Reach the target | Euclidean distance in task-space |
| `-λ * max(0, r_safe - dist)` | Stay away from obstacle | λ=10.0, r_safe=0.12 (2× obstacle radius) |

No effort penalty. One new variable only — obstacle avoidance. Clean
comparison against reaching_6dof baseline.

The obstacle penalty is a **soft barrier**: zero outside `r_safe`,
linearly increasing inside. This creates a penalty ridge around the
obstacle without discontinuities in the gradient. The policy gets
smooth gradient information about *how close* it is — not a binary
contact signal.

**Implementation**: All positions read directly from simulation state
inside the closure — no pre-captured constants needed:

- Fingertip: `data.site_xpos[1]` (`Vector3<f64>`, exact site position)
- Obstacle: `data.xpos[4]` (`Vector3<f64>`, body position — single
  source of truth, automatically correct even if the obstacle were
  ever made dynamic)
- Target: captured as `Vector3<f64>` constant (computed once via FK at
  task construction, same as `reaching_6dof`)

Distance computations use nalgebra's `(a - b).norm()` instead of
manual `dx*dx + dy*dy + dz*dz`. Both are equivalent but `.norm()` is
cleaner when both operands are already `Vector3<f64>`.

**Why distance, not contacts**: Distance gives the policy gradient
information (closer = worse, with smooth falloff). Contact count is
binary (touching or not) — no gradient signal for "almost touching."
Distance is better for learning. Contacts are better for realism.
Realism comes in 6c+1.

### Done condition

```
dist(fingertip, target) < 0.05 && vel < 1.0
```

Same as `reaching_6dof`. The task is "reach the target" — the obstacle
penalty steers behavior, it doesn't gate termination.

### Truncated condition

`data.time > 5.0` (same as reaching_6dof).

---

## 3. Predictions

### CEM vs gradient methods

| Factor | CEM | Gradient methods |
|--------|-----|-----------------|
| Obstacle penalty | Treats as black-box reward dip | Can follow penalty gradient away from obstacle |
| Multi-modal paths | Gaussian perturbations smear across both paths | Commit to one path, follow curvature |
| Task-space reward | No advantage from FK Jacobian structure | Gradient flows through obs→action mapping |
| 21-dim obs space | Same as reaching — more dims, more candidates needed | More dims → richer gradient signal |

**Prediction**: At 2-layer [64,64] with 50 envs / 50 epochs:

- TD3 and SAC should **clearly outperform CEM** (not the razor-thin
  margin of Test 10)
- PPO should outperform CEM (currently 3 OOM behind — should close to
  ~1 OOM or less)
- REINFORCE should improve relative to reaching_6dof but still trail
- CEM's absolute performance should degrade significantly vs. its
  reaching_6dof baseline

### MLP vs linear

- Linear policy on this task should plateau badly — it can learn to
  reach the target OR avoid the obstacle, but not both simultaneously
  (requires nonlinear decision boundary)
- MLP should clearly outperform linear, reversing the Test 5 finding

---

## 4. Implementation plan

### Phase 6c-0: Precursors — COMPLETE

Committed: `28bbc69` (2026-04-08).

- **6c-0a: `SiteXpos` extractor** — `SiteXpos(Range<usize>)` variant in
  `space.rs`, `SiteRangeOutOfBounds` in `error.rs`, 2 tests. ~58 LOC.
- **6c-0b: Custom task smoke test** — `tests/custom_task.rs`, CEM trains
  3 epochs on custom `TaskConfig::builder()`, reward improves. ~50 LOC.

Verification: `cargo test -p sim-ml-bridge --lib site_xpos` and
`cargo test -p sim-ml-bridge --test custom_task --release`.

### Phase 6c-1: Task function — COMPLETE

Committed: `cc83630` (2026-04-08).

Added `obstacle_reaching_6dof()` to `task.rs` as a new stock task.
The existing `reaching_6dof()` is unchanged. Exported from `lib.rs`.

**What was implemented:**

1. FK analysis determined obstacle position: rest fingertip at
   (0.750, 0, 0), target fingertip at (0.681, 0.154, 0.101). Midpoint
   (50%) was too close to target (0.098m < r_safe=0.12m). Placed at
   30% of rest→target path: (0.730, 0.046, 0.030).
2. `MJCF_6DOF_OBSTACLE` constant with `fusestatic="false"`, obstacle
   body, target site on worldbody.
3. 21-dim obs: `all_qpos()` + `all_qvel()` + `SiteXpos(1..2)` +
   `Xpos(4..5)` + `SiteXpos(0..1)`, with 21-element `obs_scale`.
4. Reward: `-dist(fingertip, target) - 10.0 * max(0, 0.12 - dist(fingertip, obstacle))`.
   Fingertip via `data.site_xpos[1]`, obstacle via `data.xpos[4]`,
   target captured as `Vector3<f64>` via FK. Distances via `.norm()`.
5. Done: `dist < 0.05 && vel < 1.0` using `data.site_xpos[1]`.
   Truncated: `data.time > 5.0`.
6. 7 unit tests: dims, build, step, reward sign, penalty fires near
   obstacle, penalty zero at target config, site ordering verified.

Also in this commit: re-exported 6 nalgebra types from `sim-core`
(`Vector3`, `UnitQuaternion`, `DVector`, `DMatrix`, `Matrix3`, `Matrix6`).

Verification: `cargo test -p sim-ml-bridge --lib obstacle` (7/7 pass).

### Phase 6c-2: Competition test — TODO

Add Test 13 (`competition_6dof_obstacle_autograd_2layer`) to
`tests/competition.rs`.

- Same setup as Test 9 (2-layer [64,64], seed 42, 50 envs, 50 epochs)
- All 5 algorithms on the obstacle avoidance task
- Task: `obstacle_reaching_6dof()` (imported from `sim_ml_bridge`)
- Assert: TD3 or SAC beats CEM
- Assert: MLP beats linear (add a linear comparison if time permits)
- Expected runtime: ~60 min in release mode

Verification: `cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture competition_6dof_obstacle`

### Phase 6c-3: Document findings — TODO (blocked on 6c-2 results)

Update `COMPETITION_TESTS_SPEC.md` with Test 13 results, analysis of
whether the ordering reversed, and comparison to Tests 8-12.

---

## 5. Evolution roadmap

6c is the first step. Each successor adds one variable and has a clean
predecessor to compare against:

| Phase | What changes | What it tests |
|-------|-------------|---------------|
| **6c** (this spec) | Distance-penalty obstacle, no contacts, no effort | Nonlinear reward landscape breaks CEM dominance |
| **6c+1** | Enable contacts (`<flag contact="enable"/>`) | Real contact forces replace ghost penalty — does the ordering hold with discontinuous dynamics? |
| **6c+2** | Add effort penalty (`-λ_effort * Σ(ctrl²)`) | Stacked nonlinear pressures — does the gradient advantage widen? |

Baby steps. Each phase ships, gets analyzed, then informs the next.

---

## 6. Open questions

1. **Obstacle position**: RESOLVED — FK analysis placed obstacle at
   (0.730, 0.046, 0.030), 30% along rest→target path. Midpoint (50%)
   was too close to target (0.098m < r_safe). At 30%: target distance
   0.138m > r_safe, rest distance 0.058m < r_safe.

2. **Obstacle size**: RESOLVED — 0.06m radius confirmed. With r_safe=0.12,
   penalty zone blocks the straight-line path while detour paths are
   feasible within joint limits. Unit tests verify penalty fires at rest
   and is zero at target.

3. **Penalty coefficient (λ=10.0)**: Locked. At rest, penalty ≈ 0.62
   (10.0 × 0.062), comparable to reaching reward magnitude (~0.14).
   Strong enough to force a detour, not so strong that reaching is
   impossible.

4. **Safe radius (r_safe=0.12)**: Locked at 2× obstacle radius. Gives
   0.018m margin at target position (0.138m - 0.12m = 0.018m).

5. **Site ordering verification**: RESOLVED — verified that worldbody
   sites are processed before child body sites (`mod.rs:280` before
   `mod.rs:287-289`), so target=site 0, fingertip=site 1. Still add
   a runtime `assert_eq!(model.nsite, 2)` in the task constructor as
   a safety check.

---

## 7. Verification

```bash
# Phase 6c-0a: SiteXpos extractor
cargo test -p sim-ml-bridge --lib site_xpos

# Phase 6c-0b: Custom task smoke test
cargo test -p sim-ml-bridge --test custom_task --release

# Phase 6c-1: Obstacle task unit tests
cargo test -p sim-ml-bridge --lib obstacle

# Phase 6c-2: Competition test (expect ~60 min in release)
cargo test -p sim-ml-bridge --test competition --release -- --ignored --nocapture competition_6dof_obstacle
```

---

## 8. Codebase stress test findings

Issues found during spec review against the codebase (2026-04-08).
Second pass (post-6c-0 implementation) on 2026-04-08 confirmed all
assumptions still hold.

### Resolved in this spec revision

| Issue | Severity | Resolution |
|-------|----------|------------|
| `fusestatic` merges obstacle body | Critical | Added `fusestatic="false"` to `<compiler>` |
| `xpos(seg3)` is 0.2m from fingertip, not 0.02m | Critical | Use `SiteXpos` extractor for true fingertip position |
| No `SiteXpos` extractor exists | Blocker | Phase 6c-0a adds it (~58 LOC across `space.rs` + `error.rs`, 8 touch points mirroring `Xpos` + new `SiteRangeOutOfBounds` error variant). **Implemented and committed: `28bbc69`.** |
| Custom `TaskConfig::builder()` never trained on | Risk | Phase 6c-0b smoke test (~50 LOC, inline MJCF, real reward, 3 epochs of CEM). **Implemented and committed: `28bbc69`.** |
| No way to observe constants (target pos) | Design | Target is a site on worldbody, observed via `SiteXpos` |
| Obstacle position captured as constant | Design | Read `data.xpos[4]` directly in closure instead — single source of truth, automatically correct if obstacle becomes dynamic |

### Verified as working

| Component | Status | Evidence |
|-----------|--------|---------|
| FK: `site_xpos` computation | Correct | `position.rs:174-184`, tested to 1e-10 |
| FK: static body `xpos` | Correct | Joint loop iterates 0 times, position = parent + offset |
| `data.forward()` atomicity | Correct | All body/site poses computed in single call |
| Zero-mass geoms | Safe | Contribute 0 to inertia, no errors |
| `contact="disable"` | Correct | Skips entire collision pipeline (`collision/mod.rs:450`) |
| Competition runner | Task-agnostic | Accepts any `TaskConfig`, reads dims at runtime |
| Algorithm builders | Generic | All 14 autograd builders use `task.obs_dim()`/`task.act_dim()`/`task.obs_scale()` — obs_dim 12→21 requires zero builder changes |
| `Xpos` extraction | Correct | Direct body indexing, extracted after `forward()` |
| `SiteXpos` extraction | Correct | **6c-0a implemented.** 301 lib tests pass, 1 integration test passes. |
| Custom task training path | Correct | **6c-0b implemented.** CEM trains 3 epochs, reward improves. |
| Site ordering (target=0, fingertip=1) | Correct | Worldbody sites processed at `mod.rs:280` before child bodies at `mod.rs:287-289`. Body sites at `body.rs:256-264` before children. |
| `fusestatic="false"` parsing | Correct | `parser.rs:477-480`, tested at `compiler.rs:352-371` |
| `model.nsite = 2` | Correct | `build.rs:66` — one entry per `<site>` element |
| `.site_xpos()` chainable + mixable with `.xpos()` | Correct | No uniqueness constraints, independent validation |
| `Vector3<f64>` in closures | Correct | `Send + Sync + 'static`, `.norm()` available |
| `data.xpos[4]` in reward closure | Correct | Public field, `&Data` passed to closure |
| `max_episode_steps` | Correct | `competition.rs:55-58` — returns 500 for act_dim > 2 |
