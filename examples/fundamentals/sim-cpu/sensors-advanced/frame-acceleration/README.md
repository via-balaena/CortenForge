# Frame Acceleration Sensors

Linear and angular acceleration at a site on a pendulum arm.

## What it demonstrates

- `<framelinacc>` includes gravity as pseudo-acceleration — at rest it reads
  [0, 0, +9.81], identical to a real accelerometer
- `<frameangacc>` reads angular acceleration (dω/dt) in world frame
- Both are acceleration-stage sensors (read from `cacc`)
- Two phases: motor hold (static) then free swing (dynamic)

## Expected visual behavior

Phase 1 (0–3s): Arm held perfectly horizontal by a strong motor. No motion.
HUD shows a = [0, 0, +9.81].

Phase 2 (3–15s): Motor releases, arm swings as a pendulum. Tip traces arcs.
HUD shows |a| oscillating, exceeding g at the bottom of each swing
(centripetal acceleration adds to gravity).

## Expected console output

```
t=  1.0s  HOLD  |a|=9.8100  alpha_y=0.0000
t=  2.0s  HOLD  |a|=9.8100  alpha_y=0.0000
t=  3.0s  SWING  |a|=11.472  alpha_y=-3.324
t=  4.0s  SWING  |a|=11.573  alpha_y=-3.537
```

## Pass/fail criteria

| Check | Condition | Tolerance |
|-------|-----------|-----------|
| Rest: a_z = +g | max error during t=1–2.5s | < 0.5% of g |
| Rest: alpha ≈ 0 | max |alpha| during t=1–2.5s | < 0.01 rad/s^2 |
| Swing: sign match | sign(alpha_Y) = sign(cos(theta)) | 0 violations |
| Swing: |a| > g | centripetal + gravity at bottom | verified |

## Run

```bash
cargo run -p example-sensor-adv-frame-acceleration --release
```
