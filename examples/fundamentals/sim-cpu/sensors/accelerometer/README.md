# Accelerometer Sensor

The classic IMU-at-rest test: proper acceleration from ground reaction force.

## What it demonstrates

- `<accelerometer>` measures proper acceleration in the sensor (site) frame
- Acceleration-stage sensor (requires full constraint solve)
- Uses `condim="1"` (frictionless normal-only contact) to avoid pyramidal
  friction forces that produce spurious lateral accelerations
- 1-second visual pause before the drop
- Two distinct phases:
  1. **Free-fall** (t < 0.7s): accel ≈ 0 (Einstein equivalence — free fall = weightless)
  2. **At-rest** (t > 5s): accel ≈ [0, 0, +9.81] (ground pushes up against gravity)

## Expected visual behavior

A red box hovers at z=4.0 for 1 second, then drops onto a grey ground plane.
It falls, impacts, bounces briefly, then settles. The HUD shows "FREE-FALL"
during the drop and "AT REST" after contact.

## Expected console output

```
t=  0.5s  accel=(+0.00,+0.00, +0.00)  |a|=0.00  expected=9.81
t=  1.0s  accel=(+0.00,+0.00,+XX.XX)  |a|=XX.XX expected=9.81
...
t=  5.0s  accel=(+0.00,+0.00, +9.XX)  |a|=9.XX  expected=9.81
```

Early readings near zero (free-fall), then oscillating around +9.81 on the
Z-axis as the box settles on the spring-damper contact.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Free-fall \|az\| < 0.5 | near-zero during fall (t=0.01–0.7s) | < 0.5 m/s² |
| Rest az ≈ +9.81 | mean Z-acceleration after settling | < 2% error |
| Rest \|a\| in [9.5, 10.1] | total magnitude reasonable | bounded |

## Run

```bash
cargo run -p example-sensor-accelerometer --release
```
