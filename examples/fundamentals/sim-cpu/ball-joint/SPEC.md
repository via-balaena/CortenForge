# Ball Joint Example — Spec

**Status:** Approved
**Date:** 2026-03-25

## Overview

A **3D spherical pendulum** demonstrating `type="ball"`, quaternion representation
(4 qpos / 3 qvel), exponential-map integration, cone limits, and `BallQuat` /
`BallAngVel` sensors.

## Scene

Two pendulums side by side, each a rigid rod (capsule) with a heavy sphere at the
tip, hanging from the world via a ball joint:

- **Pendulum A (blue, left)** — Unlimited ball joint. Displaced ~30° from vertical
  in both pitch and roll, then released. Traces a precessing elliptical path. Very
  low damping (0.001) so energy is nearly conserved.

- **Pendulum B (red, right)** — Limited ball joint, cone limit = 45°
  (`range="0 0.7854"`). Displaced ~60° from vertical (beyond the 45° limit). The
  cone constraint pushes it back inside. Same low damping.

## MJCF Model

```xml
<mujoco model="ball_joint">
    <compiler angle="radian"/>
    <option gravity="0 0 -9.81" timestep="0.001" integrator="RK4"/>

    <default>
        <geom contype="0" conaffinity="0"/>
    </default>

    <worldbody>
        <!-- Pendulum A: unlimited -->
        <body name="pend_a" pos="-0.4 0 0">
            <joint name="ball_a" type="ball" damping="0.001"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="rod_a" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.5 0.5 0.5 1"/>
            <geom name="tip_a" type="sphere" size="0.06"
                  pos="0 0 -0.5" rgba="0.2 0.5 0.85 1"/>
        </body>

        <!-- Pendulum B: limited (45° cone) -->
        <body name="pend_b" pos="0.4 0 0">
            <joint name="ball_b" type="ball" damping="0.001"
                   limited="true" range="0 0.7854"/>
            <inertial pos="0 0 -0.5" mass="1.0" diaginertia="0.01 0.01 0.01"/>
            <geom name="rod_b" type="capsule" size="0.02"
                  fromto="0 0 0  0 0 -0.5" rgba="0.5 0.5 0.5 1"/>
            <geom name="tip_b" type="sphere" size="0.06"
                  pos="0 0 -0.5" rgba="0.82 0.22 0.15 1"/>
        </body>
    </worldbody>

    <sensor>
        <ballquat name="quat_a" joint="ball_a"/>
        <ballangvel name="angvel_a" joint="ball_a"/>
        <ballquat name="quat_b" joint="ball_b"/>
        <ballangvel name="angvel_b" joint="ball_b"/>
    </sensor>
</mujoco>
```

## Initial Conditions

- **Pendulum A**: Set qpos to a quaternion representing ~30° tilt from vertical
  (rotation about an axis in the XY plane). This gives a 3D spherical pendulum
  trajectory.
- **Pendulum B**: Set qpos to a quaternion representing ~60° tilt — beyond the 45°
  cone limit. The simulation should immediately enforce the limit.

## Validation (5 checks at t=15s)

| # | Check | Method | Threshold |
|---|-------|--------|-----------|
| 1 | **Quat norm** | Track `‖q‖` for both joints every frame; report max deviation from 1.0 | < 1e-10 |
| 2 | **Energy conservation** | Track total energy (KE + PE) for pendulum A; report max drift from initial | < 0.5% of initial energy |
| 3 | **Cone limit** | Extract rotation angle from pendulum B quaternion via `atan2`; use `LimitTracker` with range `[0, pi/4]` | < 0.01 rad violation |
| 4 | **BallQuat sensor** | Compare `sensordata[adr..adr+4]` vs `qpos[qpos_adr..qpos_adr+4]` every frame; track max difference | < 1e-10 |
| 5 | **BallAngVel sensor** | Compare `sensordata[adr..adr+3]` vs `qvel[dof_adr..dof_adr+3]` every frame; track max difference | < 1e-10 |

## New Validation Trackers

The ball-joint example needs two trackers that don't exist yet. These were already
anticipated in the building blocks spec as "future trackers":

1. **`QuaternionNormTracker`** — feeds quaternion components each frame, tracks max
   deviation of `‖q‖` from 1.0.
2. **`EnergyConservationTracker`** — feeds energy each frame, tracks max absolute
   drift from the initial energy value. Reports drift as percentage of initial.
   (Different from `EnergyMonotonicityTracker` which only checks for increases.)

Both go in `sim_core::validation` alongside the existing three trackers.

## Visual

- Metallic materials: `BrushedMetal` for rods, `PolishedSteel` with blue/red tint
  for sphere tips, `CastIron` for mount points
- No ground plane — pendulums hang in space
- Camera positioned to see both pendulums from a ~45° angle
- Directional lights with shadows
- Per-second diagnostic print: time, quaternion (w,x,y,z), angular velocity
  magnitude, energy

## Diagnostics Output (per second)

```
t=  1.0s  A: q=(0.97,0.18,0.13,0.01) |w|=1.23 E=2.3456J  B: angle=44.8deg
t=  2.0s  A: q=(0.95,-.12,0.22,0.03) |w|=1.19 E=2.3451J  B: angle=43.2deg
```

## File Structure

```
examples/fundamentals/sim-cpu/ball-joint/
  Cargo.toml      # example-ball-joint
  SPEC.md         # this file
  BUGS.md         # bugs found during development
  README.md       # museum plaque
  src/main.rs     # ~350 LOC
```

## Implementation Plan

1. Add `QuaternionNormTracker` and `EnergyConservationTracker` to `sim_core::validation`
2. Create the example directory and Cargo.toml
3. Create BUGS.md (empty)
4. Baby-step 1: Simplest possible ball joint — single unlimited pendulum, just gravity, verify it swings
5. Baby-step 2: Add BallQuat/BallAngVel sensors, verify they match qpos/qvel
6. Baby-step 3: Add energy tracking, verify conservation
7. Baby-step 4: Add second pendulum with cone limit, verify limit enforcement
8. Baby-step 5: Add quaternion norm tracking
9. Baby-step 6: Polish visuals (metallic materials, camera, lights)
10. Baby-step 7: Full validation report, README
