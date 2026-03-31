# Keyframes — State Snapshots and Reset

Keyframes are named state snapshots stored in the model. A `<key>` element
in MJCF captures a complete simulation state — joint positions, velocities,
actuator activations, control signals, and mocap poses. One call to
`Data::reset_to_keyframe()` overwrites the entire simulation state and
clears all derived quantities, giving you a clean restart from any saved
pose.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [save-restore](save-restore/) | Named keyframe cycling | Pendulum snaps between 3 poses (rest, horizontal, inverted) every 3 seconds. |
| [multi-body](multi-body/) | Keyframes with ctrl state | Two-link arm with position actuators cycles through 3 poses. Control signals are part of the snapshot. |
| [stress-test](stress-test/) | Headless validation | 12 checks: bitwise-exact state restoration, derived state clearing, name lookup, deterministic replay. |

## Key ideas

- **`<key name="..." qpos="..." qvel="..."/>`** — MJCF syntax for defining a
  keyframe. All fields are optional; unspecified fields fill with model defaults.
- **`model.keyframe_id("name")`** — O(1) name-based lookup. Returns the index
  for use with `reset_to_keyframe()`.
- **`data.reset_to_keyframe(&model, idx)`** — overwrites time, qpos, qvel, act,
  ctrl, and mocap state. Clears qacc, contacts, applied forces, and sensor data.
- **`data.forward(&model)` after reset** — recomputes kinematics and derived
  quantities from the new state. Always call this after resetting.
- **ctrl is part of the snapshot.** Without ctrl in the keyframe, actuators
  would hold their previous targets and immediately drive the arm away from the
  new pose.
