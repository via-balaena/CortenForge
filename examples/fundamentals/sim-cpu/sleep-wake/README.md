# Sleep/Wake — Island-Based Deactivation

The sleep system groups bodies into **constraint islands** and deactivates
islands whose velocities fall below a threshold. Sleeping bodies skip
integration and collision narrowphase — essential for large scenes with many
resting objects.

## Examples

| Example | Concept | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | All subsystems | Headless: 18 checks against engine internals |
| [sleep-settle](sleep-settle/) | Velocity threshold | 5 boxes drop and turn blue one by one as they sleep |
| [wake-on-contact](wake-on-contact/) | Contact-triggered wake | Ball strikes sleeping box — box wakes on impact |
| [island-groups](island-groups/) | Island independence | Two stacks — poke one, the other stays asleep |

## Key ideas

- **Sleep is opt-in**: `<flag sleep="enable"/>` must be set. Off by default.
- **Velocity threshold**: a body sleeps when all DOF velocities stay below
  `sleep_tolerance * dof_length` for `MIN_AWAKE` consecutive timesteps (10).
- **Constraint islands**: the engine builds a contact/constraint adjacency
  graph each step and flood-fills connected components. Each island sleeps
  and wakes atomically — all trees in an island must be ready before any sleep.
- **Wake triggers**: contact with an awake body (`mj_wake_collision`), equality
  constraint coupling (`mj_wake_equality`), or applied force (`mj_wake` via
  `xfrc_applied`). Wake propagates through the contact graph.
- **Init-sleep**: `sleep="init"` on a body starts it asleep from timestep 0.
  The body is frozen in place — no integration, no gravity drift — until an
  external event wakes it.
- **Color convention**: all visual examples use orange = awake, steel blue =
  asleep, dark grey = static (ground).
