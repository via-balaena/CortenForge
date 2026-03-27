# FramePos + FrameQuat Sensors

FK-dependent frame sensors attached to a site on a conical pendulum (ball joint).

## What it demonstrates

- `<framepos>` reads world-frame position of a site (3D vector)
- `<framequat>` reads world-frame orientation of a site (unit quaternion)
- Both are position-stage sensors computed after forward kinematics
- Quaternion sign ambiguity: `q` and `-q` represent the same rotation
- Ball joint gives 3D motion — all 3 position components and all 4 quaternion
  components are exercised (a hinge only moves in a single plane)

## Expected visual behavior

A ball-joint pendulum tilted 30° from vertical orbits in a steady conical
trajectory. The tip traces a circle in 3D space. A site at the tip tracks
the world position and orientation as it orbits.

## Expected console output

```
t=  1.0s  pos=(+0.XXX,+0.XXX,-0.433)  r=0.250m  E=-3.5286J
t=  2.0s  pos=(+0.XXX,-0.XXX,-0.433)  r=0.250m  E=-3.5286J
...
```

X and Y oscillate as the tip orbits. Z stays constant at -L·cos(30°).
Orbit radius stays at exactly L·sin(30°) = 0.250m.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| FramePos == site_xpos | Euclidean distance to backing data | < 1e-12 |
| FrameQuat rotation dist | `1 - \|q_sensor · q_data\|` (handles sign flip) | < 1e-12 |
| Pos X range > 0.4m | orbit spans full diameter | range > 0.4 |
| Pos Y range > 0.4m | orbit spans full diameter | range > 0.4 |
| t=0 analytical pos | tip at `(0, L·sin(θ), -L·cos(θ))` | < 1e-6 |
| Energy conservation | undamped system (harness tracker) | < 0.5% drift |

## Run

```bash
cargo run -p example-sensor-frame-pos-quat --release
```
