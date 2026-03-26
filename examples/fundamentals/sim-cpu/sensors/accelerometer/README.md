# Accelerometer Sensor

The classic IMU-at-rest test: proper acceleration from ground reaction force.

## What it demonstrates

- `<accelerometer>` measures proper acceleration in the sensor (site) frame
- Acceleration-stage sensor (requires full constraint solve)
- Two distinct phases:
  1. **Free-fall** (t < 0.2s): accel ≈ 0 (Einstein equivalence — free fall = weightless)
  2. **At-rest** (t > 2s): accel ≈ [0, 0, +9.81] (ground pushes up against gravity)

## Expected visual behavior

A red box drops from z=0.5 onto a grey ground plane. It falls, impacts, bounces
briefly, then settles to rest on the surface. After settling, the box is
stationary for the remainder of the simulation.

## Expected console output

```
t=  0.0s  accel=(+0.00,+0.00, +0.00)  |a|=0.00  expected=9.81
t=  1.0s  accel=(+0.00,+0.00, +9.XX)  |a|=9.XX  expected=9.81
t=  2.0s  accel=(+0.00,+0.00, +9.81)  |a|=9.81  expected=9.81
...
```

Early readings near zero (free-fall), then converging to +9.81 on the Z-axis.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Free-fall \|az\| < 0.5 | near-zero during fall (t=0.01–0.2s) | < 0.5 m/s² |
| Rest az ≈ +9.81 | mean Z-acceleration after settling | < 2% error |
| Rest \|a\| in [9.5, 10.1] | total magnitude reasonable | bounded |

## Run

```bash
cargo run -p example-sensor-accelerometer --release
```
