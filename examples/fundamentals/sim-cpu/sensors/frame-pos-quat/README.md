# FramePos + FrameQuat Sensors

FK-dependent frame sensors attached to a site on a swinging pendulum.

## What it demonstrates

- `<framepos>` reads world-frame position of a site (3D vector)
- `<framequat>` reads world-frame orientation of a site (unit quaternion)
- Both are position-stage sensors computed after forward kinematics
- Quaternion sign ambiguity: `q` and `-q` represent the same rotation

## Expected visual behavior

A hinge pendulum displaced 30° from vertical swings back and forth. A site at
the pendulum tip tracks the tip's world position and orientation as it swings.

## Expected console output

```
t=  1.0s  pos=(+0.XXX,+0.000,-0.XXX)  q=(+0.XXX,+0.000,+0.XXX,+0.000)
...
```

The X-component of position oscillates as the tip swings laterally. The
Y-component stays near zero (hinge rotates in the XZ plane).

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| FramePos == site_xpos | Euclidean distance to backing data | < 1e-12 |
| FrameQuat rotation dist | `1 - \|q_sensor · q_data\|` (handles sign flip) | < 1e-12 |
| Pos X range > 0.2m | tip actually swings laterally | range > 0.2 |
| t=0 analytical pos | tip at `(L*sin(30°), 0, -L*cos(30°))` | < 1e-6 |
| Energy conservation | undamped system (harness tracker) | < 0.5% drift |

## Run

```bash
cargo run -p example-sensor-frame-pos-quat --release
```
