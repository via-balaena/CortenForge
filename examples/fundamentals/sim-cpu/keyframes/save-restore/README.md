# Save-Restore — Pendulum Keyframe Cycling

A single pendulum arm snaps between three named poses every 3 seconds.

## What you're seeing

A rigid arm hangs from a hinge joint. Every 3 seconds it resets to the next
keyframe in the cycle: **rest** (hanging straight down), **horizontal** (arm
out to the side, motionless), **inverted** (arm straight up, motionless).
Between resets the arm swings freely under gravity.

## What to look for

- **The snap.** Each reset is instantaneous. The arm teleports to the saved
  pose — there is no interpolation, no animation blend. This is a full state
  replacement: position, velocity, and simulation time are all overwritten in
  one call.
- **Velocity matters.** The rest and horizontal keyframes have zero velocity,
  so the arm starts motionless after those resets. Watch how quickly gravity
  pulls it away from horizontal versus how gently it swings from rest.
- **The inverted pose.** Straight up is an unstable equilibrium. After reset
  the arm balances for a brief moment, then tips and swings down. The
  direction it falls depends on floating-point noise — this is correct.
- **The HUD.** Shows the current keyframe name, time since last reset, joint
  angle (qpos), and angular velocity (qvel). Watch qpos and qvel jump to
  exact keyframe values at each reset.

## The physics

A hinge joint has 1 degree of freedom: `qpos[0]` is the angle (radians),
`qvel[0]` is the angular velocity. A keyframe stores a complete snapshot of
these values. `Data::reset_to_keyframe()` overwrites the simulation state
and clears all derived quantities (acceleration, contacts, applied forces),
so the simulation restarts cleanly from the saved pose.

The three keyframes:
- **rest:** qpos=pi/2 (hanging down), qvel=0
- **horizontal:** qpos=0 (arm out to the right), qvel=0
- **inverted:** qpos=-pi/2 (arm straight up), qvel=0

## Run

```
cargo run -p example-keyframes-save-restore --release
```
