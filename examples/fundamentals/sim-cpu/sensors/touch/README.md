# Touch Sensor

Contact force measurement via the touch sensor.

## What it demonstrates

- `<touch>` sums normal contact forces on all geoms of the sensor's body
- Acceleration-stage sensor (reads constraint forces from solver)
- `condim="1"` (frictionless, normal-only) avoids pyramidal friction approximation
  which would report ~75% of the true normal force
- Positive data kind: touch is always >= 0
- 1-second visual pause before the drop

## Expected visual behavior

A blue sphere hovers at z=4.0 for 1 second, then drops onto a grey ground
plane. It falls, impacts, bounces a few times, then settles. The HUD shows
"FREE-FALL" during the drop and "CONTACT" after impact.

## Expected console output

```
t=  0.5s  touch=  0.00 N  contact=no   expected=9.81 N
t=  1.0s  touch=  9.XX N  contact=yes  expected=9.81 N
...
t=  5.0s  touch=  9.81 N  contact=yes  expected=9.81 N
```

Touch is exactly 0 during free-fall, then converges to m*g = 9.81 N at rest.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Free-fall touch=0 | exactly zero during fall (t=0.01–0.7s) | < 1e-10 |
| Rest touch ≈ mg | mean force after settling (t > 5s) | < 5% error |
| Non-negative | touch >= 0 every single frame | 0 violations |

## Run

```bash
cargo run -p example-sensor-touch --release
```
