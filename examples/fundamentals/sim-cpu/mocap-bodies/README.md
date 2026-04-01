# Mocap Bodies — Kinematic Input Bodies

Mocap (motion capture) bodies are world-attached bodies whose pose is set
directly via `data.mocap_pos` and `data.mocap_quat` rather than computed by
the integrator. They are the mechanism for user input, teleop, animation
playback, and VR controllers.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | Headless validation | 12 checks: position/quaternion tracking, gravity immunity, contact generation, keyframe restore, weld-to-mocap, zero-mass FK, child-follows-parent. |
| [drag-target](drag-target/) | Weld-to-mocap tracking | Green ghost sphere glides on a sine wave; a box chases it with springy lag via a compliant weld constraint. |
| [push-object](push-object/) | Mocap contact interaction | Red paddle sweeps across the scene and flicks a blue ball through one-way contact forces. |
| [spin-fling](spin-fling/) | Orientation-driven interaction | Orange turntable spins via `mocap_quat`; friction flings a ball off the edge. |

## Key ideas

- **`mocap="true"`:** Declared on a `<body>` element in MJCF. Must be a
  direct child of worldbody. Cannot have joints.
- **`data.mocap_pos` / `data.mocap_quat`:** User-settable arrays indexed by
  mocap ID (not body ID). Modified between `step()` calls to drive poses.
- **Kinematic, not dynamic:** Mocap bodies are positioned by FK override —
  gravity, contact forces, and constraint forces do not affect them.
- **One-way interaction:** Mocap geoms participate in collision, but contact
  forces only act on the dynamic side. Mocap bodies push things without
  being pushed back.
- **Weld-to-mocap:** A compliant weld constraint (`solref` for stiffness)
  connects a dynamic body to a mocap body. The dynamic body follows with
  spring-like lag — the standard "drag target" pattern for teleop and VR.
- **Keyframe restore:** `reset_to_keyframe()` restores `mocap_pos` and
  `mocap_quat` from the keyframe's `mpos`/`mquat` fields.
