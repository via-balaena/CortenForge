# Keyframe Examples Spec

**Date:** 2026-03-31
**Status:** Draft
**Track:** 1B — sim-core foundation layer 2
**Parent:** `examples/COVERAGE_SPEC.md`, section "keyframes/"

## Overview

Keyframes are named state snapshots stored in the model (`<key>` elements in
MJCF). They let you save and restore complete simulation states — position,
velocity, activation, control, and mocap state. These examples demonstrate
keyframe definition, name-based lookup, and reset-to-keyframe as a simulation
control primitive.

## Examples

### 1. `save-restore/` — Pendulum Reset Cycle (visual)

A single-link pendulum swings under gravity. Every 3 seconds it resets to the
next keyframe in a cycle of three named poses: **rest** (hanging straight down),
**horizontal** (arm out, zero velocity), **inverted** (arm straight up, zero
velocity). Each reset snaps the arm instantly to the saved pose.

**MJCF model:**
- Single hinge joint, 1 DOF
- 3 keyframes: `rest` (qpos=0), `horizontal` (qpos=pi/2), `inverted` (qpos=pi)
- All keyframes: qvel=0, time=0

**What the user sees:**
- Pendulum swinging, then snapping to a new pose every 3 seconds
- HUD showing: current keyframe name, time since reset, qpos, qvel
- Console prints state before/after each reset

**Concepts demonstrated:**
- `<key name="..." qpos="..."/>` MJCF syntax
- `model.keyframe_id("name")` for name-based lookup
- `data.reset_to_keyframe(&model, idx)` for state restoration
- `data.forward(&model)` required after reset
- Cycling through multiple keyframes by name

### 2. `multi-body/` — Two-Link Arm with Control State (visual)

A two-link planar arm with 2 hinge joints and 2 position actuators. Three
keyframes represent distinct arm configurations: **home** (both joints at 0,
zero ctrl), **reach-left** (joint 1 at -pi/3, joint 2 at pi/4, ctrl matching),
**reach-right** (mirror of reach-left). Cycles through keyframes every 3
seconds.

**MJCF model:**
- 2 hinge joints (shoulder + elbow), 2 position actuators
- 3 keyframes with `qpos` and `ctrl` fields populated
- Demonstrates keyframe with both joint state and control state

**What the user sees:**
- Arm snapping between three poses
- HUD showing: current keyframe name, qpos[0..2], ctrl[0..2]
- The arm settles into each pose because ctrl matches the keyframe's target

**Concepts demonstrated:**
- Keyframes with `ctrl` field — control state is part of the snapshot
- Multi-DOF keyframes (qpos length > 1)
- Position actuators driven to keyframe-specified targets
- How ctrl and qpos interact after reset (actuator drives to target)

### 3. `stress-test/` — Headless Validation (implemented first)

Headless test exercising every keyframe code path. No Bevy, no window.

**Checks (12 total):**

#### State restoration (checks 1-5)
1. **qpos exact match** — After `reset_to_keyframe`, `data.qpos` matches the
   keyframe's qpos to machine precision (f64 bitwise).
2. **qvel exact match** — Same for qvel.
3. **ctrl exact match** — Model with actuator. After stepping (ctrl mutated by
   control callback or manual write), reset restores ctrl to keyframe value.
4. **act exact match** — Model with a stateful actuator (dyntype=integrator).
   After stepping (mutating act), reset restores act to keyframe value.
5. **time exact match** — Keyframe with `time="2.5"`. After reset, `data.time`
   is exactly 2.5.

#### Derived state cleared (checks 6-7)
6. **qacc zeroed** — After stepping (qacc nonzero) then resetting, qacc is all
   zeros.
7. **contacts cleared** — Model with a contact-producing setup. After stepping
   (contacts > 0) then resetting, `data.ncon == 0` and contacts is empty.

#### Multiple keyframes (checks 8-9)
8. **Index correctness** — Model with 3 named keyframes. Each
   `reset_to_keyframe(i)` loads the correct state (verified by distinct qpos
   values per keyframe).
9. **Name lookup correctness** — Same model. `model.keyframe_id("name")` returns
   the correct index for all 3, and the index produces the correct state after
   reset.

#### Edge cases (checks 10-11)
10. **Zero-length fields** — Model with no actuators (na=0, nu=0). Keyframe
    with only qpos/qvel. Reset succeeds without panic.
11. **Reset mid-simulation** — Step 500 times, reset, step 500 more. Final
    state matches a fresh run of 500 steps from the keyframe (deterministic
    replay).

#### Post-reset simulation (check 12)
12. **Simulation runs normally after reset** — Reset to keyframe (Model C,
    free joint), then step 1000 times. Verify energy is finite, qpos is
    finite, quaternion norm preserved. No NaN, no divergence.

## File structure

```
examples/fundamentals/sim-cpu/keyframes/
  README.md
  save-restore/
    Cargo.toml
    README.md
    src/main.rs
  multi-body/
    Cargo.toml
    README.md
    src/main.rs
  stress-test/
    Cargo.toml
    src/main.rs
```

## Stress test MJCF models

Three minimal inline models, each designed for specific checks:

**Model A — Single hinge, 3 named keyframes (checks 1-2, 5, 6, 8-9, 11):**
```xml
<mujoco model="keyframe-hinge">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002" integrator="RK4">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0.5 0 0" mass="1.0"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="rest"       qpos="0.0"    qvel="0.0" time="0.0"/>
    <key name="horizontal" qpos="1.5708" qvel="0.5" time="2.5"/>
    <key name="inverted"   qpos="3.1416" qvel="-1.0" time="5.0"/>
  </keyframe>
</mujoco>
```

**Model B — Hinge + position actuator with dyntype=integrator (checks 3-4):**
```xml
<mujoco model="keyframe-actuated">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.05" fromto="0 0 0 0.5 0 0" mass="1.0"/>
    </body>
  </worldbody>
  <actuator>
    <general name="motor" joint="hinge" gainprm="10" biasprm="0 -10 0"
             dyntype="integrator" dynprm="1"/>
  </actuator>
  <keyframe>
    <key name="home" qpos="0.5" qvel="0.0" act="0.3" ctrl="1.0"/>
  </keyframe>
</mujoco>
```

**Model C — Free body + floor, contact (checks 7, 12):**
```xml
<mujoco model="keyframe-contact">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002">
    <flag energy="enable"/>
  </option>
  <worldbody>
    <geom name="floor" type="plane" size="5 5 0.1"/>
    <body name="ball" pos="0 0 0.5">
      <freejoint name="free"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="above" qpos="0 0 0.5 1 0 0 0"/>
  </keyframe>
</mujoco>
```

**Model D — No actuators, hinge only (check 10 zero-length fields):**
```xml
<mujoco model="keyframe-no-actuator">
  <compiler angle="radian"/>
  <option gravity="0 0 -9.81" timestep="0.002"/>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="sphere" size="0.1" mass="1.0"/>
    </body>
  </worldbody>
  <keyframe>
    <key name="tilted" qpos="0.5" qvel="1.0"/>
  </keyframe>
</mujoco>
```

## Implementation order

1. **stress-test** — first. Validates every code path before we build visuals.
2. **save-restore** — simplest visual. Single DOF, single concept.
3. **multi-body** — adds ctrl/actuator dimension.

## Workspace registration

Add to root `Cargo.toml` members:
```toml
"examples/fundamentals/sim-cpu/keyframes/stress-test",
"examples/fundamentals/sim-cpu/keyframes/save-restore",
"examples/fundamentals/sim-cpu/keyframes/multi-body",
```

## Acceptance criteria summary

| # | Check | Model | Tolerance |
|---|---|---|---|
| 1 | qpos exact after reset | A | bitwise (0.0 drift) |
| 2 | qvel exact after reset | A | bitwise |
| 3 | ctrl exact after reset | B | bitwise |
| 4 | act exact after reset | B | bitwise |
| 5 | time exact after reset | A | bitwise |
| 6 | qacc zeroed after reset | A | < 1e-15 |
| 7 | contacts cleared after reset | C | ncon == 0 |
| 8 | 3 keyframes indexed correctly | A | bitwise per keyframe |
| 9 | name lookup → correct index → correct state | A | bitwise |
| 10 | zero-length act/ctrl fields | D | no panic |
| 11 | deterministic replay after mid-sim reset | A | < 1e-12 |
| 12 | no divergence after reset | C | all finite, quat norm < 1e-10 drift |
