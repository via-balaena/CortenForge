# Clock Sensor

The simplest possible sensor: reads `data.time` directly.

## What it demonstrates

- The `<clock/>` sensor type (position-stage, no target object)
- Sensor pipeline runs and produces fresh data every frame

## Expected visual behavior

A hinge pendulum swings back and forth (provides visual life — the clock sensor
itself has no visual representation). The pendulum is undamped and swings
indefinitely.

## Expected console output

```
t=  1.0s  clock=1.00000  time=1.00000
t=  2.0s  clock=2.00000  time=2.00000
...
```

The clock sensor value matches `data.time` to machine precision every line.

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Clock == time | sensor matches `data.time` every frame | < 1e-15 |
| Monotonic | clock never decreases between frames | 0 violations |
| Clock > 14s | proves time actually advanced by report time | clock > 14.0 |
| Energy conservation | undamped pendulum (harness tracker) | < 0.5% drift |

## Run

```bash
cargo run -p example-sensor-clock --release
```
