# Multi-Body — Two-Link Arm with Control State

A two-link planar arm snaps between three named poses every 3 seconds.

## What you're seeing

A shoulder-elbow arm made of two capsules connected by hinge joints. Each
joint has a position actuator that drives it toward a target angle. Every 3
seconds the arm resets to the next keyframe in the cycle: **home** (both
joints at zero — arm extended right), **reach-left** (shoulder rotated
forward, elbow bent back), **reach-right** (mirror of reach-left).

## What to look for

- **The snap.** Just like save-restore, each reset is instantaneous — both
  joint angles and both control signals jump to the keyframe values in a
  single call.
- **The arm settles.** After each reset the position actuators are already
  set to targets that match the keyframe pose. The arm starts in the right
  position with the right control, so it holds steady (minor oscillation
  from gravity).
- **ctrl is part of the snapshot.** This is the key difference from
  save-restore. Without ctrl in the keyframe, the actuators would still
  hold their previous targets and immediately drive the arm away from the
  new pose. With ctrl included, the arm stays put.
- **The HUD.** Shows the current keyframe name, both joint angles
  (qpos[0], qpos[1]), and both control signals (ctrl[0], ctrl[1]).
  Watch all four values jump simultaneously at each reset.

## The physics

A position actuator applies torque proportional to the error between its
control input (`ctrl`) and the current joint angle. The gain and bias
parameters determine the stiffness and damping:

```
force = gain * (ctrl - qpos) + bias * qvel
```

When a keyframe sets both `qpos` and `ctrl` to the same target angle, the
error is zero at reset — so the actuator applies no torque and the arm
holds position. Gravity causes slight drift, which the actuator corrects.

The three keyframes:
- **home:** shoulder=0, elbow=0, both ctrl=0 (arm extended right)
- **reach-left:** shoulder=pi/3, elbow=-pi/4, ctrl matching
- **reach-right:** shoulder=-pi/3, elbow=pi/4, ctrl matching

## Run

```
cargo run -p example-keyframes-multi-body --release
```
