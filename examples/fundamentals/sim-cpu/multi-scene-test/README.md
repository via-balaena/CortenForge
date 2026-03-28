# Multi-Scene Visual Test

Infrastructure test for the `MultiScenePlugin` — three identical pendulums
side by side, each in its own independent physics scene. Proves that
multi-scene spawning, lockstep stepping, and per-scene offset sync all work.

## What you see

Three pendulums spaced 1.5 units apart (red, green, blue tips), swinging in
perfect unison. Same initial conditions + same physics = identical motion.

## What it tests

- `PhysicsScenes::add()` — creating multiple independent scenes
- `step_scenes_lockstep()` — all scenes advance by the same dt
- `spawn_scene_geoms()` — per-scene entity spawning with world-space offset
- Per-scene geom sync — transforms track the correct scene's data

## Run

```
cargo run -p example-multi-scene-test --release
```
