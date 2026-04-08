# Phase 6c — Obstacle Avoidance Task

> **Status**: Draft — decisions locked, ready for implementation
> **Crate**: sim-ml-bridge
> **Parent spec**: AUTOGRAD_SPEC.md (Phase 6c), COMPETITION_TESTS_SPEC.md
> **Branch**: main

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
    <site name="target" pos="TBD" size="0.015"/>
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
    <body name="obstacle" pos="TBD">
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

**Obstacle placement** (TBD — requires FK analysis):

The obstacle goes roughly midway between the arm's rest-state fingertip
position and the target fingertip position. The exact position must be
determined empirically by:

1. Computing rest-state fingertip position (all qpos = 0)
2. Computing target fingertip position (qpos = target_joints)
3. Placing the obstacle at the midpoint, possibly offset slightly to
   ensure the straight-line path is blocked but both "go around" paths
   are feasible

The obstacle radius (0.06 m) should be large enough that the arm can't
squeeze past without detouring, but small enough that the detour is
achievable within the joint limits. This will require tuning — see
Open Questions.

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

Both distance computations use `data.site_xpos[1]` (fingertip) in the
reward/done closures — exact position, consistent with what the policy
observes via `SiteXpos`.

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

### Phase 6c-0: Precursors (de-risk before building the task)

Two small, self-contained changes that 6c depends on. Ship and test
these first. Both are purely mechanical — no design decisions.

#### 6c-0a: `SiteXpos` extractor

Add a `SiteXpos(Range<usize>)` variant to `space.rs`. Mirrors the
existing `Xpos` extractor exactly — reads `data.site_xpos[site_id]`
instead of `data.xpos[body_id]`. ~58 lines across 2 files.

**`space.rs` — 8 touch points** (same as Xpos):

| # | Location | What | Lines |
|---|----------|------|-------|
| 1 | `Extractor` enum (line ~37) | `SiteXpos(Range<usize>)` variant | 1 |
| 2 | `Extractor::dim()` (line ~59) | `Self::SiteXpos(r) => r.len() * 3` | 1 |
| 3 | `Extractor::extract()` (line ~80) | Loop over `data.site_xpos[site]`, 3 floats each | 8 |
| 4 | `ObservationSpaceBuilder` (line ~335) | `.site_xpos(Range<usize>)` builder method + doc comment | 5 |
| 5 | `extractor_label()` (line ~481) | `"site_xpos({}..{})"` format string | 1 |
| 6 | `validate_extractor()` (line ~514) | `check_site("site_xpos", r, model.nsite)` | 1 |
| 7 | Tests | `extract_site_xpos()` — extract from pendulum model | ~16 |
| 8 | Tests | `site_xpos_range_out_of_bounds()` — OOB validation | ~11 |

**`error.rs` — new error variant + validation helper**:

The existing pattern has separate error variants per index space:
- `BodyRangeOutOfBounds { field, range, nbody }` for body-indexed fields
- `MocapRangeOutOfBounds { field, range, nmocap }` for mocap-indexed fields

Reusing `check_body()` for sites would produce `"nbody = 2"` in the
error message when it means `"nsite = 2"`. Following the established
pattern:

| # | Location | What | Lines |
|---|----------|------|-------|
| 9 | `error.rs` | `SiteRangeOutOfBounds { field, range, nsite }` variant | ~7 |
| 10 | `space.rs` | `fn check_site(field, range, nsite)` helper | ~7 |

**No shorthand needed.** There is no `all_xpos()` — per-body/site
extractors all use explicit ranges. No `all_site_xpos()` needed.

**Total: ~58 lines** (24 in space.rs logic, 7 in error.rs, 27 in tests).

#### 6c-0b: Custom task training smoke test

Prove that `TaskConfig::builder()` works end-to-end through actual
training — not just build + reset. The builder path has a
`builder_roundtrip()` unit test (`task.rs:679`) that validates
construction and reset, but **no test has ever run a custom-built
TaskConfig through an Algorithm**. The code path is identical to
stock tasks under the hood, but this has never been exercised.

| Item | Detail |
|------|--------|
| File | `sim/L0/ml-bridge/tests/custom_task.rs` (new integration test, auto-discovered by Cargo — no config changes) |
| MJCF | Inline ~20 lines (private `MJCF_2DOF` in `task.rs` is not exported) |
| Task | Custom `TaskConfig::builder()` with real reward (`-Σ(qpos - target)²`), not constant zero — proves the reward closure actually executes and CEM responds |
| Algorithm | `Cem::new(LinearPolicy::new(...), CemHyperparams { ... })` |
| Training | 3 epochs, 4 envs via `cem.train(&mut env, TrainingBudget::Epochs(3), 42, &|_| {})` |
| Asserts | `metrics.len() == 3`, `mean_reward.is_finite()`, reward improves (last > first) |
| Size | ~50 lines |
| Why | De-risks the entire 6c competition test. If this fails, we find out in 5 seconds, not 60 minutes. |

**Imports needed** (all publicly exported from `sim_ml_bridge`):

```rust
use sim_ml_bridge::{
    Algorithm, Cem, CemHyperparams, LinearPolicy, TrainingBudget,
    TaskConfig, ObservationSpace, ActionSpace,
};
use std::sync::Arc;
```

### Phase 6c-1: Task function

Add `obstacle_reaching_6dof()` to `task.rs` as a new stock task.

1. Determine obstacle position via FK analysis (run the sim, print
   rest-state and target fingertip positions, choose midpoint)
2. Build MJCF string with `fusestatic="false"`, obstacle body, target
   site
3. Implement observation space using `SiteXpos` (fingertip, target)
   and `Xpos` (obstacle)
4. Implement reward (distance to target + obstacle proximity penalty)
   using `data.site_xpos[1]` for fingertip position
5. Implement done/truncated (same as reaching_6dof, using
   `data.site_xpos[1]`)
6. Unit tests: dims, build, step, reward sign, obstacle penalty fires
   when fingertip is near obstacle, penalty is zero when far

### Phase 6c-2: Competition test

Add Test 13 (`competition_6dof_obstacle_autograd_2layer`) to
`tests/competition.rs`.

- Same setup as Test 9 (2-layer [64,64], seed 42, 50 envs, 50 epochs)
- All 5 algorithms on the obstacle avoidance task
- Assert: TD3 or SAC beats CEM
- Assert: MLP beats linear (add a linear comparison if time permits)

### Phase 6c-3: Document findings

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

1. **Obstacle position**: Needs FK analysis. Where exactly to place it?
   Midpoint of rest→target fingertip path is the starting hypothesis.

2. **Obstacle size**: 0.06m radius is a starting guess. Too small =
   easy to avoid (CEM might still win). Too large = impossible to reach
   target. May need a quick sweep (0.04, 0.06, 0.08) to find the sweet
   spot where the task is solvable but requires a real detour.

3. **Penalty coefficient (λ=10.0)**: Too high = policy is
   obstacle-averse and never reaches target. Too low = policy ignores
   obstacle and CEM wins by brute force. The ratio of penalty to
   reaching reward at the obstacle boundary should make touching the
   obstacle worse than a ~0.1m detour.

4. **Safe radius (r_safe=0.12)**: 2× obstacle radius. Defines the
   penalty field width. Narrower = sharper cliff (harder to learn).
   Wider = more gentle (easier to learn but weaker nonlinearity signal).

5. **Site ordering verification**: The spec assumes target site = 0,
   fingertip site = 1 based on document-order processing. Must verify
   after MJCF parsing — a quick `assert_eq!(model.nsite, 2)` and
   position check in the task constructor.

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

Issues found during spec review against the codebase (2026-04-08):

### Resolved in this spec revision

| Issue | Severity | Resolution |
|-------|----------|------------|
| `fusestatic` merges obstacle body | Critical | Added `fusestatic="false"` to `<compiler>` |
| `xpos(seg3)` is 0.2m from fingertip, not 0.02m | Critical | Use `SiteXpos` extractor for true fingertip position |
| No `SiteXpos` extractor exists | Blocker | Phase 6c-0a adds it (~58 LOC across `space.rs` + `error.rs`, 8 touch points mirroring `Xpos` + new `SiteRangeOutOfBounds` error variant) |
| Custom `TaskConfig::builder()` never trained on | Risk | Phase 6c-0b smoke test (~50 LOC, inline MJCF, real reward, 3 epochs of CEM) |
| No way to observe constants (target pos) | Design | Target is a site on worldbody, observed via `SiteXpos` |

### Verified as working

| Component | Status | Evidence |
|-----------|--------|---------|
| FK: `site_xpos` computation | Correct | `position.rs:174-184`, tested to 1e-10 |
| FK: static body `xpos` | Correct | Joint loop iterates 0 times, position = parent + offset |
| `data.forward()` atomicity | Correct | All body/site poses computed in single call |
| Zero-mass geoms | Safe | Contribute 0 to inertia, no errors |
| `contact="disable"` | Correct | Skips entire collision pipeline (`collision/mod.rs:450`) |
| Competition runner | Task-agnostic | Accepts any `TaskConfig`, reads dims at runtime |
| Algorithm builders | Generic | All 14 autograd builders use `task.obs_dim()`/`task.act_dim()` |
| `Xpos` extraction | Correct | Direct body indexing, extracted after `forward()` |
