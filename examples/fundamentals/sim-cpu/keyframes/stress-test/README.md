# Stress Test — Keyframe Validation

Headless validation of every keyframe code path. No window, no Bevy.

## What it checks

12 checks across 4 MJCF models:

**State restoration (checks 1-5):** After `reset_to_keyframe`, every primary
state field matches the keyframe value to bitwise precision — qpos, qvel,
ctrl, act, and time. Not "close to" — identical. This proves `reset_to_keyframe`
is a true memory copy, not an approximation.

**Derived state cleared (checks 6-7):** After stepping (which populates qacc,
contacts, and sensor data) then resetting, all derived quantities are zeroed.
No stale acceleration or phantom contacts leak through a reset.

**Multiple keyframes (checks 8-9):** A model with 3 named keyframes. Each
index loads the correct distinct state. Name lookup via `keyframe_id()` and
`name2id(ElementType::Keyframe, ...)` both return the correct index and
produce the correct state after reset.

**Edge cases (checks 10-11):** A model with no actuators (na=0, nu=0) resets
without panic. A mid-simulation reset produces exactly the same trajectory
as a fresh start from the same keyframe — deterministic replay.

**Post-reset stability (check 12):** A free-joint body resets above a floor,
then runs 1000 steps. All state remains finite, quaternion norm is preserved,
energy is bounded. No NaN, no divergence.

## Run

```
cargo run -p example-keyframes-stress-test --release
```
