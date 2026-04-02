# Sensors Advanced — Remaining Sensor Types

Advanced sensor examples covering all sensor types not demonstrated in the
Track 1A `sensors/` gallery. One concept per example, each with analytical
validation checks.

## Examples

| Example | Sensors | What you see |
|---------|---------|-------------|
| [stress-test](stress-test/) | All 15 types | Headless: 25 checks against analytical predictions |
| [frame-velocity](frame-velocity/) | FrameLinVel, FrameAngVel | Rod spinning at 1 rev/s — |v|=π, ω_z=2π |
| [frame-acceleration](frame-acceleration/) | FrameLinAcc, FrameAngAcc | Pendulum held then released — a=[0,0,+g] at rest |
| [force-torque](force-torque/) | Force, Torque | Static beam — F=mg, τ=mgL/2 |
| [rangefinder](rangefinder/) | Rangefinder | Sphere bobbing above ground — tracks height, -1 for no-hit |
| [subtree-velocity](subtree-velocity/) | SubtreeLinVel | 3-link chain free-fall — v_z = -g*t |
| [subtree-angmom](subtree-angmom/) | SubtreeAngMom | Spinning rod in zero-g — |L| conserved to machine epsilon |
| [actuator-pos-vel](actuator-pos-vel/) | ActuatorPos, ActuatorVel | Gear=2 servo — ratio locked at exactly 2.000000 |

## Coverage

After this directory, **31/31 non-plugin sensor types** have dedicated
examples across the codebase. See the
[spec](SENSORS_ADVANCED_SPEC.md) for the full coverage map.
