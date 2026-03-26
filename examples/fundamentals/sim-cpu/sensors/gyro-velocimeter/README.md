# Gyro + Velocimeter Sensors

Velocity-stage sensors reading in the sensor's local frame on a conical
pendulum (ball joint).

## What it demonstrates

- `<gyro>` measures angular velocity in the sensor (site) frame
- `<velocimeter>` measures linear velocity in the sensor (site) frame
- Both are velocity-stage sensors (computed after velocity FK)
- Ball joint gives multi-axis readings — a hinge would only produce 1
  non-zero component per sensor
- For a conical pendulum (tilt θ, precession Ω around world Z):
  - Gyro: `[0, Ω·sin(θ), Ω·cos(θ)]` — constant, 2 non-zero components
  - Velocimeter: `[-L·Ω·sin(θ), 0, 0]` — tangential velocity in sensor frame

## Expected visual behavior

A ball-joint pendulum tilted 30° orbits steadily around the vertical axis.
The tip traces a circle in 3D space. Both gyro and velocimeter magnitudes
remain constant throughout the orbit.

## Expected console output

```
t=  1.0s  gyro=(+0.000,+2.380,+4.122)  |ω|=4.760  |v|=1.190
t=  2.0s  gyro=(+0.000,+2.380,+4.122)  |ω|=4.760  |v|=1.190
...
```

Gyro Y and Z are constant and non-zero. |ω| = Ω (precession rate).
|v| = Ω·L·sin(θ) (tip orbital speed). All values remain constant
because the conical orbit is steady-state.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Gyro == qvel (3D) | all 3 components match joint angular velocity | < 1e-10 |
| \|gyro\| == Ω | magnitude equals analytical precession rate | < 1e-4 |
| \|veloc\| == Ω·L·sin(θ) | magnitude equals analytical tip speed | < 1e-4 |
| Gyro Y non-trivial | \|gyro_y\| > 1.0 rad/s | > 1.0 |
| Gyro Z non-trivial | \|gyro_z\| > 1.0 rad/s | > 1.0 |
| Energy conservation | undamped system (harness tracker) | < 0.5% drift |

## Run

```bash
cargo run -p example-sensor-gyro-velocimeter --release
```
