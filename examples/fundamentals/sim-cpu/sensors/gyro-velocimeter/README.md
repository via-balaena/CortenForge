# Gyro + Velocimeter Sensors

Velocity-stage sensors reading in the sensor's local frame.

## What it demonstrates

- `<gyro>` measures angular velocity in the sensor (site) frame
- `<velocimeter>` measures linear velocity in the sensor (site) frame
- Both are velocity-stage sensors (computed after velocity FK)
- For a Y-axis hinge, exact analytical relationships hold:
  - Gyro: `[0, ω, 0]` — rotation axis is invariant under Y-rotation
  - Velocimeter: `[-Lω, 0, 0]` — tip always moves in local -X (tangent to arc)

## Expected visual behavior

A hinge pendulum displaced 30° swings back and forth. A site at the tip
carries both sensors. The pendulum is undamped and swings indefinitely.

## Expected console output

```
t=  1.0s  gyro=(+0.000,+2.XXX,+0.000)  vel=(-1.XXX,+0.000,+0.000)  ω=+2.XXX
t=  2.0s  gyro=(+0.000,-1.XXX,+0.000)  vel=(+0.XXX,+0.000,+0.000)  ω=-1.XXX
...
```

Gyro Y-component matches ω exactly. Velocimeter X = -0.5 * ω. All other
components stay at zero.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Gyro Y == ω | matches `joint_qvel[0]` | < 1e-10 |
| Gyro X,Z == 0 | off-axis components are zero | < 1e-10 |
| Veloc X == -Lω | matches `-0.5 * joint_qvel[0]` | < 1e-10 |
| Veloc Y,Z == 0 | off-axis components are zero | < 1e-10 |
| Gyro Y range | angular velocity spans meaningful range | > 1.0 rad/s |
| Energy conservation | undamped system (harness tracker) | < 0.5% drift |

## Run

```bash
cargo run -p example-sensor-gyro-velocimeter --release
```
